from typing import List, Union
from qtm_tools import QTMBodyData
import numpy as np
import cflib.crazyflie


def waypoint_manager(body: QTMBodyData,
                     cf: cflib.crazyflie.Crazyflie,
                     waypoint_number: Union[int, None],
                     uav_num: int,
                     waypoints: List[List[float]],
                     synchronize_waypoints: bool,
                     all_on_waypoint: bool,
                     send_packet: bool,
                     on_waypoint: List[bool],
                     sequence_activated):

    """ Controls each cf according to the provided trajectories description and the UAVs synchronicity """

    # Maximum distance (m) between a cf and a waypoint for which the waypoint is considered to be reached
    radius_threshold: float = 0.15

    if abs(body.roll) > 45 or abs(body.pitch) > 45 or body.z > 1.5:
        print(' ---- Warning ----', body.body_name, ': uncontrolled attitude detected, emergency stop')
        print('                   -> Roll =', body.roll, '° ; Pitch =', body.pitch, '° ; Z =', body.z, 'm')
        waypoint_number = None

    if waypoint_number is not None:
        position_goal = waypoints[waypoint_number]
        if position_goal[0] is None:
            position_goal[0] = body.x
        if position_goal[1] is None:
            position_goal[1] = body.y
        if position_goal[2] is None:
            position_goal[2] = body.z
        if position_goal[3] is None:
            position_goal[3] = body.yaw

        if position_goal[2] != 0:
            distance = np.sqrt(
                (body.x - position_goal[0]) ** 2 + (body.y - position_goal[1]) ** 2 + (body.z - position_goal[2]) ** 2)
        else:
            # For flight stability reasons, if a landing waypoint is encountered,
            # then the distance criteria is only based on the cf height
            # -> Even if the cf lands outside the expected waypoint sphere,
            #    the motors are turned off as soon as it approaches the ground
            distance = np.sqrt((body.z - position_goal[2]) ** 2)

        if send_packet:
            # print('Distance =', distance)
            cf.commander.send_position_setpoint(position_goal[0], position_goal[1], position_goal[2], position_goal[3])
        #     cf.commander.send_setpoint(0, 0, 0, 1)

        if distance < radius_threshold:
            # print(body.body_name, 'on waypoint')
            if sequence_activated or waypoint_number == len(waypoints) - 1:
                if synchronize_waypoints:
                    on_waypoint[uav_num] = True
                    # If all UAVs have reached their respective waypoints, then they can move to the next ones
                    if all_on_waypoint:
                        waypoint_number, waypoints = update_waypoint(waypoint_number, waypoints, body, cf)
                else:
                    waypoint_number, waypoints = update_waypoint(waypoint_number, waypoints, body, cf)
    else:
        # A None waypoint_number either means that the corresponding cf has finished its flight trajectory,
        # or that a problem occurred (QTM tracking lost for example)
        # For both cases, let's make sure that the motors are stopped
        cf.commander.send_stop_setpoint()

    return waypoint_number, waypoints, on_waypoint


def update_waypoint(waypoint_number: int,
                    waypoints: List[List[float]],
                    body: QTMBodyData,
                    cf: cflib.crazyflie.Crazyflie):
    if waypoint_number < len(waypoints) - 1:
        waypoint_number = waypoint_number + 1
        if waypoints[waypoint_number][0] is None:
            waypoints[waypoint_number][0] = body.x
        if waypoints[waypoint_number][1] is None:
            waypoints[waypoint_number][1] = body.y
        if waypoints[waypoint_number][2] is None:
            waypoints[waypoint_number][2] = body.z
        if waypoints[waypoint_number][3] is None:
            waypoints[waypoint_number][3] = body.yaw
        print(body.body_name, ': Moving towards waypoint', waypoint_number, '@', waypoints[waypoint_number])
        cf.commander.send_position_setpoint(waypoints[waypoint_number][0],
                                            waypoints[waypoint_number][1],
                                            waypoints[waypoint_number][2],
                                            waypoints[waypoint_number][3])

    else:
        waypoint_number = None
        print(body.body_name, ': Last waypoint reached, end of sequence')
        cf.commander.send_stop_setpoint()

    return waypoint_number, waypoints
