#!/usr/bin/env python3

# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


""" This client uses a neural network to predict the throttle and brake. These
predictions are then used as control signals, but the steer is still controled
by the autopilot.

Run: python3.5 client_controller_test.py -c CarlaSettings.ini"""

from __future__ import print_function

import argparse
import random
import logging
import random
import time
import math
import numpy as np
import matlab.engine
import scipy.io

from carla.client import make_carla_client
import save_util as saver
from carla.sensor import Camera, Lidar
from carla.settings import CarlaSettings
from carla.tcp import TCPConnectionError
from carla.util import print_over_same_line


def run_carla_client(args):

    number_of_total_frames = 0
    frames_per_episode = 1800
    max_total_frames = 36000
    episode = 0

    print('Starting matlab engine')
    eng = matlab.engine.start_matlab()
    print('Started.')

    matlab_wd = '/home/annaochjacob/Repos/carla/PythonClient/MPC_drive/data/throttle_measurements/'
    eng.addpath(matlab_wd)

    with make_carla_client(args.host, args.port) as client:
        print('client connected')

        throttle = 0.0

        #for episode in range(0, number_of_episodes):
        while number_of_total_frames < max_total_frames:
            episode += 1

            settings = CarlaSettings()
            settings.set(
                SynchronousMode=False,
                SendNonPlayerAgentsInfo=True,
                NumberOfVehicles=0,
                NumberOfPedestrians=0,
                WeatherId=random.choice([1]),
                QualityLevel=args.quality_level)
            settings.randomize_seeds()

            camera0 = Camera('CameraRGB')
            camera0.set_image_size(800, 600)
            camera0.set_position(0.30, 0, 1.30)
            settings.add_sensor(camera0)

            scene = client.load_settings(settings)
            number_of_player_starts = len(scene.player_start_spots)
            player_start = random.randint(0, max(0, number_of_player_starts - 1))
            #2, 34
            print('Starting new episode...')
            client.start_episode(player_start)

            # Create matrix for holding time, acceleration and throttle, velocity
            values = np.zeros([1,22+4])

            i = 0
            choose = 1
            # Iterate every frame in the episode.
            for frame in range(0, frames_per_episode):

                # Read the data produced by the server this frame.
                measurements, sensor_data = client.read_data()
                control = measurements.player_measurements.autopilot_control
                pm = saver.get_player_measurements(measurements)

                heading_angle = pm[11]
                steer = pm[17]
                acceleration_x = pm[5]
                acceleration_y = pm[6]
                acc_xy = np.array([acceleration_x, acceleration_y])
                forward_speed = pm[8]

                theta = np.atan(acc_xy*acc_xy/acc_xy)
                current_acc_v = np.linalg.norm(acc_xy) * np.cos(heading_angle - theta)

                #compare accelerations:
                print("magnitude:")
                print(np.linalg.norm(current_acc_v), desired_acc_magnitude)
                print('vectors:')
                print(current_acc_v, desired_acc_v)

                if frame % 30 == 0:
                    rand = random.random()
                    choose = random.random()

                if choose > 0.3:
                    # accelerate
                    desired_acc_magnitude = rand
                else:
                    # decelerate
                    desired_acc_magnitude = -rand

                # Calculate x and y components to match desired acc
                desired_acc_vx = desired_acc_magnitude * np.cos(np.deg2rad(heading_angle))
                desired_acc_vy = desired_acc_magnitude * np.sin(np.deg2rad(heading_angle))
                desired_acc_v = [desired_acc_vx, desired_acc_vy]
                predicted_throttle, predicted_brake = eng.eval_net_python(float(heading_angle),
                                                                          float(steer),
                                                                          float(desired_acc_vx),
                                                                          float(desired_acc_vy),
                                                                          float(forward_speed), nargout=2)
                print(desired_acc_magnitude, predicted_throttle, predicted_brake)

                # Set throttle and brake to the predicted values
                control.throttle = predicted_throttle
                control.brake = predicted_brake

                # override values
                pm[17] = control.steer
                pm[18] = control.throttle
                pm[19] = control.brake
                pm[20] = control.hand_brake
                pm[21] = control.reverse

                client.send_control(control)

                # Insert values into table for saving
                predicted = np.array([desired_acc_x, desired_acc_y, predicted_throttle, predicted_brake])
                pm = np.concatenate((pm,predicted))
                pm = np.expand_dims(pm,axis=1).transpose()
                values = np.concatenate([values, pm], axis=0)

                is_colliding = measurements.player_measurements.collision_other > 0.0

                if is_colliding: break
                number_of_total_frames += 1

            # Save measured values
            filename = '/media/annaochjacob/crucial/recorded_data/carla/OTHER2/controller_test/episode_%i.csv' % episode
            header = saver.get_player_measurements_header()
            header = header + ',desired_acc_x,desired_acc_y,predicted_throttle,predicted_brake'
            values = values[1:,:]
            np.savetxt(filename, values, delimiter=',', comments='', fmt='%.4f', header=header)


def print_measurements(measurements):
    number_of_agents = len(measurements.non_player_agents)
    player_measurements = measurements.player_measurements
    message = 'Vehicle at ({pos_x:.1f}, {pos_y:.1f}), '
    message += '{speed:.0f} km/h, '
    message += 'Collision: {{vehicles={col_cars:.0f}, pedestrians={col_ped:.0f}, other={col_other:.0f}}}, '
    message += '{other_lane:.0f}% other lane, {offroad:.0f}% off-road, '
    message += '({agents_num:d} non-player agents in the scene)'
    message = message.format(
        pos_x=player_measurements.transform.location.x,
        pos_y=player_measurements.transform.location.y,
        speed=player_measurements.forward_speed * 3.6, # m/s -> km/h
        col_cars=player_measurements.collision_vehicles,
        col_ped=player_measurements.collision_pedestrians,
        col_other=player_measurements.collision_other,
        other_lane=100 * player_measurements.intersection_otherlane,
        offroad=100 * player_measurements.intersection_offroad,
        agents_num=number_of_agents)
    print_over_same_line(message)

def get_values(measurements):
    player_measurements = measurements.player_measurements
    acc_x = player_measurements.acceleration.x
    acc_y = player_measurements.acceleration.y
    acc_z = player_measurements.acceleration.z
    acceleration = math.sqrt(acc_x**2 + acc_y**2)# + acc_z**2)
    yaw = player_measurements.transform.rotation.yaw
    x = player_measurements.transform.location.x
    y = player_measurements.transform.location.y
    z = player_measurements.transform.location.z
    velocity = player_measurements.forward_speed
    time = measurements.game_timestamp
    return time, acceleration, velocity, yaw,x,y,z


def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('-v', '--verbose',action='store_true',dest='debug',
                            help='print debug information')
    argparser.add_argument('--host',metavar='H',default='localhost',
                            help='IP of the host server (default: localhost)')
    argparser.add_argument('-p', '--port',metavar='P',default=2000,type=int,
                            help='TCP port to listen to (default: 2000)')
    argparser.add_argument('-q', '--quality-level',choices=['Low', 'Epic'],type=lambda s: s.title(),default='Epic',
                            help='graphics quality level, a lower level makes the simulation run considerably faster.')
    argparser.add_argument('-i', '--images-to-disk',action='store_true',dest='save_images_to_disk',
                            help='save images (and Lidar data if active) to disk')
    argparser.add_argument('-c', '--carla-settings',metavar='PATH',dest='settings_filepath',default=None,
                            help='Path to a "CarlaSettings.ini" file')

    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    args.out_filename_format = '_out/episode_{:0>4d}/{:s}/{:0>6d}'

    while True:
        try:
            run_carla_client(args)
            print('Done.')
            return

        except TCPConnectionError as error:
            logging.error(error)
            time.sleep(1)


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
