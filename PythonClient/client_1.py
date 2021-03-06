#!/usr/bin/env python3

# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Basic CARLA client example."""

from __future__ import print_function

import argparse
import logging
import random
import time
import os

from carla.client import make_carla_client
from carla.sensor import Camera
from carla.settings import CarlaSettings
from carla.tcp import TCPConnectionError
from carla.util import print_over_same_line
import numpy as np
import save_util as saver
from collections import defaultdict
import datetime
import torch
from network import SmallerNetwork2
from pure_pursuit import State, PIDControl, pure_pursuit_control, calc_target_index
import client_util
from torch.autograd import Variable

argparser = argparse.ArgumentParser(description=__doc__)
argparser.add_argument(
    '-v', '--verbose',
    action='store_true',
    dest='debug',
    help='print debug information')
argparser.add_argument(
    '--host',
    metavar='H',
    default='localhost',
    help='IP of the host server (default: localhost)')
argparser.add_argument(
    '-p', '--port',
    metavar='P',
    default=2000,
    type=int,
    help='TCP port to listen to (default: 2000)')
argparser.add_argument(
    '-a', '--autopilot',
    action='store_true',
    help='enable autopilot')
argparser.add_argument(
    '-i', '--images-to-disk',
    action='store_true',
    help='save images to disk')
argparser.add_argument(
    '-c', '--carla-settings',
    metavar='PATH',
    default=None,
    help='Path to a "CarlaSettings.ini" file')
argparser.add_argument(
    '--frames',
    default=100,
    type=int,
    dest='frames',
    help='Number of frames to run the client')
argparser.add_argument(
    '--save-path',
    metavar='PATH',
    default='recorded_data/',
    dest='save_path',
    help='Number of frames to run the client')
argparser.add_argument(
    '-pl', '--planner-path',
    metavar='PATH',
    default=None,
    dest='planner_path',
    help='Path to planner checkpoint')

args = argparser.parse_args()

SAVE_PATH_PLAYER = args.save_path + 'player_measurements/'
SAVE_PATH_STATIC = args.save_path + 'static_measurements/'
SAVE_PATH_DYNAMIC = args.save_path + 'dynamic_measurements/'
SAVE_PATH_PREDICTIONS = args.save_path + 'generated_output/'
SAVE_PATH_POINT_CLOUD = args.save_path + 'point_cloud/'
SAVE_PATH_RGB_IMG = args.save_path + 'rgb_images/'
ANNOTATIONS_PATH = '/media/annaochjacob/crucial/annotations/path_2018_03_27/'
N_PAST_STEPS = 30
MOVING_AVERAGE_LENGTH = 10
STEERING_MAX = 70 # angle in degrees


def run_carla_client(host, port, autopilot_on, planner_path, save_images_to_disk, image_filename_format, settings_filepath):


    # We assume the CARLA server is already waiting for a client to connect at
    # host:port. To create a connection we can use the `make_carla_client`
    # context manager, it creates a CARLA client object and starts the
    # connection. It will throw an exception if something goes wrong. The
    # context manager makes sure the connection is always cleaned up on exit.
    with make_carla_client(host, port) as client:
        print('CarlaClient connected')

        if settings_filepath is None:

            # Create a CarlaSettings object. This object is a wrapper around
            # the CarlaSettings.ini file. Here we set the configuration we
            # want for the new episode.
            settings = CarlaSettings()
            settings.set(
                SynchronousMode=True,
                SendNonPlayerAgentsInfo=True,
                NumberOfVehicles=20,
                NumberOfPedestrians=40,
                WeatherId=random.choice([1, 3, 7, 8, 14]))
            settings.randomize_seeds()

            # Now we want to add a couple of cameras to the player vehicle.
            # We will collect the images produced by these cameras every
            # frame.

            # The default camera captures RGB images of the scene.
            camera0 = Camera('CameraRGB')
            # Set image resolution in pixels.
            camera0.set_image_size(800, 600)
            # Set its position relative to the car in centimeters.
            camera0.set_position(30, 0, 130)
            settings.add_sensor(camera0)

            # Let's add another camera producing ground-truth depth.
            camera1 = Camera('CameraDepth', PostProcessing='Depth')
            camera1.set_image_size(800, 600)
            camera1.set_position(30, 0, 130)
            settings.add_sensor(camera1)

        else:

            # Alternatively, we can load these settings from a file.
            with open(settings_filepath, 'r') as fp:
                settings = fp.read()

        # Now we load these settings into the server. The server replies
        # with a scene description containing the available start spots for
        # the player. Here we can provide a CarlaSettings object or a
        # CarlaSettings.ini file as string.
        scene = client.load_settings(settings)

        # Choose one player start at random.
        number_of_player_starts = len(scene.player_start_spots)
        player_start = 56

        # Notify the server that we want to start the episode at the
        # player_start index. This function blocks until the server is ready
        # to start the episode.
        print('Starting new episode...')
        client.start_episode(player_start)

        episode_start = datetime.datetime.now()
        print("Episode started %s" % episode_start.strftime("%Y-%m-%d %H:%M"))
        print("Space required: %.2fGB" %(args.frames*1.1/1024))
        print("---------------------------------------------------------------")
        # Create placeholders for measurement values
        player_values = np.zeros([args.frames,22])
        dynamic_values = defaultdict(list)
        static_values = []
        #Create container array for moving average
        frame_durations = MOVING_AVERAGE_LENGTH * [0]

        # Load checkpoint
        if planner_path is not None: #we have predictions
            model = get_inference_model(planner_path)
            print('MODEL LOADED')

            intentions_path = np.genfromtxt( ANNOTATIONS_PATH + 'intentions_path.csv', names=True, delimiter=' ')
            traffic_path = np.genfromtxt( ANNOTATIONS_PATH + 'traffic_path.csv', names=True, delimiter=' ')
            # Create container for values in previous steps, plus one to make use
            # of existing code
            prev_measurements = np.zeros([N_PAST_STEPS + 1, 22])
            prev_intentions = np.zeros([N_PAST_STEPS + 1, 2]) #direction, distance
            prev_traffic = np.zeros([N_PAST_STEPS + 1, 4]) #distance, current_limit, next_limit, light_status

        # Iterate every frame in the episode.
        for frame in range(0,args.frames):
            # Time processing of each frame to estimate episode duration
            start = time.time()

            # Read the data produced by the server this frame.
            measurements, sensor_data = client.read_data()

            # Append player measurements for this frame
            pm = saver.get_player_measurements(measurements)
            player_values[frame,:] = pm

            # Append non-player measurements for this frame
            dynamic_objects = saver.get_dynamic_measurements(measurements)
            for key, value in dynamic_objects.items():
                dynamic_values[key].append(value)

            # If first frame, get static info about non-player agents and save
            # to csv file. Also, get header for dynamic measurements
            if frame == 0:
                static_values = saver.get_static_measurements(measurements)
                saver.save_static_measurements(static_values, SAVE_PATH_STATIC)
                # Create csv header for objects with dynamic states
                header_dynamic = saver.get_dynamic_measurements_header(measurements)

                if planner_path is not None:
                    #find all traffic light ids and stuff.
                    traffic_path = client_util.find_traffic_sign_ids(static_values, traffic_path)
                    prev_measurements = np.tile(pm, (N_PAST_STEPS +1, 1))

                    #TODO init intentions and traffic
                    prev_intentions = np.tile([intentions_path[0][2],intentions_path[0][3]],
                        (N_PAST_STEPS + 1, 1))
                    traffic_id = traffic_path['id'][0]
                    isTrafficLight = True if static_values[traffic_id]['type'] == 3 else False
                    if isTrafficLight:
                        prev_traffic = np.tile([traffic_path['next_distance'][0],30,0,0],
                            (N_PAST_STEPS + 1, 1))
                    else:
                        prev_traffic =  np.tile([traffic_path['next_distance'][0],
                            30,static_values[traffic_id]['speed_limit'],0], (N_PAST_STEPS +1, 1))
                    #[next_distance, current_speed_limit, next_speed_limit, light_status],

            # Save front facing RGB camera images
            if save_images_to_disk:
                rgb_image = sensor_data['CameraRGB']
                rgb_image.save_to_disk(\
                    image_filename_format.format(1, "CameraRGB", frame))

            # Save point cloud from current frame
            point_cloud = saver.save_point_cloud(frame, sensor_data, \
                SAVE_PATH_POINT_CLOUD)



            # Now we have to send the instructions to control the vehicle.
            # If we are in synchronous mode the server will pause the
            # simulation until we send this control.
            if planner_path is not None:
                # Roll previous measurements to behave like a FIFO queue, then insert pm
                prev_measurements = np.roll(prev_measurements,-1,0)
                prev_measurements[N_PAST_STEPS,:] = pm
                print(prev_measurements[N_PAST_STEPS,:])

                #Calculate intentions
                intention, flag = client_util.GetIntention(prev_measurements, prev_intentions, intentions_path)
                # Roll previous measurements to behave like a FIFO queue, then insert pm
                prev_intentions = np.roll(prev_intentions,1,0)
                prev_intentions[0,:] = intention
                print(intention)
                if flag: # delete first row of path
                    intentions_path = np.delete(intentions_path,0,0)

                #Calculate traffic
                traffic, flag = client_util.GetTraffic(prev_measurements, static_values, dynamic_values, prev_traffic, traffic_path)
                print(traffic)
                # Roll previous measurements to behave like a FIFO queue, then insert pm
                prev_traffic = np.roll(prev_traffic,1,0)
                prev_traffic[0,:] = traffic
                if flag: # delete first row of path
                    traffic_path = np.delete(traffic_path,0,0)

                # Create point cloud
                lidar = client_util.get_max_elevation(point_cloud)
                # Create values vector for N_PAST_STEPS backwards in time.
                # Since prev_measurements is N_PAST_STEPS, we can say that the current
                # frame has index N_PAST_STEPS. Then get_input() will use the N_PAST_STEPS
                # previous measurements available in prev_measurements.
                # TODO Use actual info for traffic and intentions instead of None
                input_values = client_util.get_input(prev_measurements, prev_intentions, prev_traffic,
                    N_PAST_STEPS, N_PAST_STEPS)

                lidars = Variable(torch.cuda.FloatTensor(lidar))
                values = Variable(torch.cuda.FloatTensor(input_values))
                lidars = lidars.view(-1, 1, 600, 600)
                values = values.view(-1, 30, 11)

                # Run model
                output = model(lidars, values)
                #output = output.data.cpu().numpy().squeeze()
                output = output.data.view(-1,2).cpu().numpy()
                print(output)
                # Separate output into x and y
                rel_x = output[:,0]
                rel_y = output[:,1]

                client_util.generate_output(frame, [rel_x, rel_y], SAVE_PATH_PREDICTIONS)

                #rel_x = np.zeros(30)
                #rel_y = -np.arange(0,30)

                # Get current state
                x, y, yaw, v = pm[2], pm[3], pm[11], pm[8]
                state = State(x, y, yaw, v/3.6)

                # Transform from relative coordinates back into CARLA world coordinates
                world_x, world_y = client_util.relative_to_world(x, -y, yaw, rel_x, rel_y)

                lastIndex = len(world_x) - 1
                target_ind = calc_target_index(state, world_x, world_y)

                # Calculate steer and throttle using pure pursuit
                print("x: %.3f, y: %.3f, yaw: %.3f, v: %.3f" %(x,y,yaw,v))

                s = client_util.getEulerDistance((rel_x[0], rel_y[0]),(rel_x[29], rel_y[29]))
                target_speed = s/(3)
                print(target_speed)
                target_speed = target_speed*3.6
                print(target_speed)
                brake = 0.0
                #target_speed = 30.0/3.6 # km/h
                if v > target_speed:
                    brake = 0.5

                # ai is the difference between current speed and target speed
                ai = PIDControl(target_speed, state.v) # (More of a P controller...)

                # di is the desired steering angle in radians
                di, target_ind = pure_pursuit_control(state, world_x, world_y, target_ind)
                # Ensure we stick to car's physical limits
                di = min(di,np.deg2rad(STEERING_MAX))
                di = max(di,np.deg2rad(-STEERING_MAX))
                print("throttle: %.3f, brake: %.3f, steer: %.3f, target_ind: %.3f" %(ai/target_speed, brake,
                    np.rad2deg(di), target_ind))

                client.send_control(
                    steer=np.rad2deg(di)/STEERING_MAX,
                    throttle= ai/target_speed,
                    brake=brake,
                    hand_brake=False,
                    reverse=False)

                #client.send_control(
                #    steer=random.uniform(-0.1, 0.1),
                #    throttle=0.5,
                #    brake=0.0,
                #    hand_brake=False,
                #    reverse=False)

            elif autopilot_on:
                # Together with the measurements, the server has sent the
                # control that the in-game autopilot would do this frame. We
                # can enable autopilot by sending back this control to the
                # server. We can modify it if wanted, here for instance we
                # will add some noise to the steer.
                control = measurements.player_measurements.autopilot_control
                #control.steer += random.uniform(-0.1, 0.1)
                client.send_control(control)
            else:
                client.send_control(
                    steer=random.uniform(-0.1, 0.1),
                    throttle=0.5,
                    brake=0.0,
                    hand_brake=False,
                    reverse=False)

            # Print estimated time of arrival
            stop = time.time()
            elapsed_time = stop - start
            frame_durations[frame%MOVING_AVERAGE_LENGTH] = elapsed_time
            average_time = np.mean(frame_durations)
            time_left = average_time * (args.frames-frame)
            m, s = divmod(time_left, 60)
            h, m = divmod(m, 60)
            print("Frame %i - ETA %02d:%02d:%02d" % (frame, h, m, s))

        # Save measurements of whole episode to one file
        saver.save_player_measurements(player_values, SAVE_PATH_PLAYER)
        saver.save_dynamic_measurements(header_dynamic, dynamic_values, \
            SAVE_PATH_DYNAMIC)

        episode_end = datetime.datetime.now()
        episode_time = episode_end - episode_start
        print("Episode ended %s, duration %s" \
            % (episode_end.strftime("%Y-%m-%d %H:%M"), episode_time))

def get_inference_model(filename):
    # Check if checkpoint exists
    if os.path.isfile(filename):
        print("Loading model at '{}'".format(filename))
    else:
        print("No file found at '{}'".format(filename))
        return None

    # Load checkpoint
    checkpoint = torch.load(filename)
    model = SmallerNetwork2()
    model.cuda()
    model.load_state_dict(checkpoint['state_dict'])
    del checkpoint
    print('Inference model successfully loaded!')
    return model

def print_measurements(measurements):
    number_of_agents = len(measurements.non_player_agents)
    player_measurements = measurements.player_measurements
    message = 'Vehicle at ({pos_x:.1f}, {pos_y:.1f}), '
    message += '{speed:.2f} km/h, '
    message += 'Collision: {{vehicles={col_cars:.0f}, pedestrians={col_ped:.0f}, other={col_other:.0f}}}, '
    message += '{other_lane:.0f}% other lane, {offroad:.0f}% off-road, '
    message += '({agents_num:d} non-player agents in the scene)'
    message = message.format(
        pos_x=player_measurements.transform.location.x / 100, # cm -> m
        pos_y=player_measurements.transform.location.y / 100,
        speed=player_measurements.forward_speed,
        col_cars=player_measurements.collision_vehicles,
        col_ped=player_measurements.collision_pedestrians,
        col_other=player_measurements.collision_other,
        other_lane=100 * player_measurements.intersection_otherlane,
        offroad=100 * player_measurements.intersection_offroad,
        agents_num=number_of_agents)
    print_over_same_line(message)


def main():
    # Create directories
    if not os.path.exists(SAVE_PATH_PLAYER):
        os.makedirs(SAVE_PATH_PLAYER)
    if not os.path.exists(SAVE_PATH_STATIC):
        os.makedirs(SAVE_PATH_STATIC)
    if not os.path.exists(SAVE_PATH_DYNAMIC):
        os.makedirs(SAVE_PATH_DYNAMIC)
    if not os.path.exists(SAVE_PATH_POINT_CLOUD):
        os.makedirs(SAVE_PATH_POINT_CLOUD)
    if not os.path.exists(SAVE_PATH_RGB_IMG):
        os.makedirs(SAVE_PATH_RGB_IMG)

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    while True:
        try:

            run_carla_client(
                host=args.host,
                port=args.port,
                autopilot_on=args.autopilot,
                planner_path=args.planner_path,
                save_images_to_disk=args.images_to_disk,
                image_filename_format= SAVE_PATH_RGB_IMG + 'episode_{:0>3d}/{:s}/image_{:0>5d}.png',
                settings_filepath=args.carla_settings)

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
