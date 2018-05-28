import numpy as np
import matplotlib.pyplot as plt
import util
import math
import os

ELEVATION_MAX = 6
ELEVATION_MIN = -18
RADIUS=3

def main():
    pm_path = '/media/annaochjacob/crucial/recorded_data/carla/train/apple/player_measurements/pm.csv'
    output_path = '/media/annaochjacob/crucial/dataset/eukaryote/train/apple/right_intention/output/output_10000.csv'
    m_output = np.genfromtxt(output_path, delimiter=',', skip_header=True)
    m_output = np.transpose(m_output)
    m_player = np.genfromtxt(pm_path, delimiter=',', skip_header=True)
    start = 10000

    print(m_output[0])

    x, y, yaw = m_player[start,[2, 3, 11]]
    x,y,yaw = util.correct_carla_coordinates(x, y, yaw)
    print(x,y,yaw)

    plt.figure()

    # Plot output in dataset (constructed using old get_relative_location)
    plt.scatter(m_output[0], m_output[1], marker='x', c='r')

    for i in range(start,(start+30)):
        # Plot actual path
        new_x, new_y = m_player[i, [2, 3]]
        new_x, new_y, _ = util.correct_carla_coordinates(new_x, new_y, yaw)
        plt.scatter(new_x, new_y, marker='.', c='g')

        # Plot the relative coordinates, transformed using world_to_relative
        rel_x, rel_y = util.world_to_relative(x, y, yaw, [new_x, new_y])
        plt.scatter(rel_x, rel_y, marker='.', c='b')

    plt.show()

def get_input(measurements, intentions, traffic, frame, n_steps):
    all_inputs = np.zeros([n_steps, 11]) # Create container for past measurements
    x, y, yaw = measurements[frame,[2, 3, 11]]

    # NOTE Carla coordinate correction
    yaw = -yaw
    y = -y

    for past_step in range(0,n_steps):
        # Get index of past frames, i.e. exluding the current frame
        frame_index = frame - past_step - 1
        # If requested frame is further into the past than frame 0, use 0
        frame_index = max(frame_index,0)
        # Calculate relative location, forward acceleration etc.
        new_x, new_y = measurements[frame_index, [2, 3]]

        new_y = -new_y # NOTE carla coordinate correction

        # Notice the minus signs on y and new_y because of carla's world axes!
        v_rel = world_to_relative(x, y, yaw, [new_x, new_y])
        acc_x, acc_y, acc_z = measurements[frame_index, [5, 6, 7]]
        v_total_acceleration = get_total_acceleration(acc_x, -acc_y, acc_z) # -acc_y because carla have reversed y axis.
        v_forward_speed = measurements[frame_index, 8]
        v_steer, v_throttle, v_break = measurements[frame_index, [17, 18, 19]]

        # Insert values in this frame's row
        frame_input = np.zeros(11)
        frame_input[0] = v_rel[0] # location x relative to car
        frame_input[1] = v_rel[1] # location y relative to car
        frame_input[2] = v_total_acceleration # forward acceleration
        frame_input[3] = v_forward_speed # forward speed
        frame_input[4] = v_steer # steer
        frame_input[5] = intentions[frame_index][0] # intention direction
        frame_input[6] = intentions[frame_index][1] # intention
        frame_input[7] = traffic[frame_index][0] # next_traffic_object_proximity
        frame_input[8] = traffic[frame_index][1] # current_speed_limit
        frame_input[9] = np.nan_to_num(traffic[frame_index][2]) # next_speed_limit (MIGHT BE NULL!)
        frame_input[10] = np.nan_to_num(traffic[frame_index][3]) # traffic light status (MIGHT BE NULL!)

        all_inputs[past_step,:] = np.transpose(frame_input)

    return all_inputs

def get_total_acceleration(acc_x, acc_y, acc_z):
    squares = np.power([acc_x, acc_y, acc_z], 2)
    return np.sqrt(np.sum(squares))

def GetIntention(player_measurements, prev_intentions, intentions_path):
    """used for turn intentions"""
    next_value_flag = False
    if intentions_path.size == 0:
        return [0, 100], next_value_flag #if path is empy, turn nowhere in 100 meters. (default values)

    next_pos = (intentions_path['location_x'][0],intentions_path['location_y'][0])
    next_value = prev_intentions[0][0] #direction in col 0
    next_distance = prev_intentions[0][1] # distance in col 1
    last_position = (player_measurements[-2][2],player_measurements[-2][3]) # col 2 and 3
    current_position = (player_measurements[-1][2],player_measurements[-1][3])

    if (next_pos == None or isWithinRadius(current_position,next_pos,RADIUS)):
        next_pos, next_value = (intentions_path['location_x'][1],intentions_path['location_y'][1])
        next_value = intentions_path['intention_direction'][1]
        next_value_flag = True
        next_distance = intentions_path['next_distance'][1]

    # get updated turn proximity
    next_distance = next_distance - getEulerDistance(last_position, current_position)
    next_distance = max(next_distance, 0)

    return [next_value, next_distance], next_value_flag #if got next intention, reutnr true so reciever can relete row.

def GetTraffic(player_measurements, static_measurements, dynamic_measurements, prev_traffic, traffic_path):
    """use for traffic"""
    next_value_flag = False
    if len(traffic_path['id']) == 0:
        return [0, prev_traffic[0][1], 0, 0], next_value_flag #if path is empy, return some default values

    next_pos = (traffic_path['location_x'][0],traffic_path['location_y'][0])
    next_id = traffic_path['id'][0]
    next_distance = prev_traffic[0][0] # distance in col 0
    last_position = (player_measurements[-2][2],player_measurements[-2][3]) # col 2 and 3
    current_position = (player_measurements[-1][2],player_measurements[-1][3])
    isTrafficLight = True if static_measurements[next_id]['type'] == 3 else False
    current_speed_limit = prev_traffic[0][1]
    next_speed_limit = prev_traffic[0][2]

    if (next_pos == None or isWithinRadius(current_position,next_pos,RADIUS)):
        current_speed_limit = next_speed_limit #can this ever be none?
        next_pos = (traffic_path['location_x'][1],traffic_path['location_y'][1])
        next_id = traffic_path['id'][1]
        next_value_flag = True
        next_distance = traffic_path['next_distance'][1]
        isTrafficLight = True if static_measurements[next_id]['type'] == 3 else False

    # get updated turn proximity
    next_distance = next_distance - getEulerDistance(last_position, current_position)
    next_distance = max(next_distance, 0)

    light_status = None
    next_speed_limit = None
    if isTrafficLight:
        light_status = dynamic_measurements[next_id][-1] #get current status for traffic light
    else:
        next_speed_limit = static_measurements[next_id]['speed_limit']

    return [next_distance, current_speed_limit, next_speed_limit, light_status], next_value_flag

def isWithinRadius(a, b, r = 3):
    if getEulerDistance(a,b) < r:
        return True
    return False

def getEulerDistance(pos_a, pos_b):
    (a_lat,a_lon) = pos_a
    (b_lat,b_lon) = pos_b
    dist = math.sqrt((b_lat - a_lat)**2 + (b_lon - a_lon)**2)
    return dist


def generate_output(frame, output, path):
    if not os.path.exists(path):
        os.makedirs(path)

    output = np.transpose(output)
    filename = path + 'gen_%i.csv' %(frame)
    np.savetxt(filename, output, comments='', delimiter=',',fmt='%.8f',
                    header='x,y')

def find_traffic_sign_ids(static_measurements, traffic_path):
    updated_traffic_path = {'id':[], 'location_x':[], 'location_y':[],'next_distance':[]}

    for idx, sign in enumerate(traffic_path):
        annotation_location = (sign[0], sign[1])
        for row in static_measurements:
            sign_location = (static_measurements[row]['location_x'], static_measurements[row]['location_y'])
            if isWithinRadius(annotation_location, sign_location, r = 2):
                updated_traffic_path['id'].append(row)
                updated_traffic_path['location_x'].append(sign[0])
                updated_traffic_path['location_y'].append(sign[1])
                updated_traffic_path['next_distance'].append(sign[2])
                break
    if not len(updated_traffic_path['id']) == len(traffic_path):
        print("COULD NOT LOCATE ALL TRAFFIC LIGHTS/SIGNS. PLEASE CHECK ANNOTATIONS.")
        print(traffic_path)
        print(updated_traffic_path)
    return updated_traffic_path


if __name__ == '__main__':
    main()
