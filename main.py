import asyncio
import qtm
import qtm_tools
import cf_tools
import flight_sequence
import pynput
# import time
from typing import List


async def start_qtm_streaming(connection: qtm.QRTConnection):
    """ Starts a QTM stream, and assigns a callback method to run each time a QRTPacket is received from QTM
     This method is made to run forever in an asyncio event loop """
    print('QTM streaming started')
    await connection.stream_frames(components=['6deuler'], on_packet=packet_reception_callback)


def cf_connected_callback(link_uri):
    global CONNECTED_CF_ADDRESSES
    global CONNECTED_CF
    global CF_READY

    for cf_index in range(len(CONNECTED_CF_ADDRESSES)):
        if CONNECTED_CF_ADDRESSES[cf_index][1] == link_uri:
            setup_ok = cf_tools.reset_kalman_estimation(CONNECTED_CF[cf_index], CONNECTED_CF_ADDRESSES[cf_index][0])
            if setup_ok:
                CF_READY[cf_index] = True
                print(CONNECTED_CF_ADDRESSES[cf_index][0], ': connection setup successful')
            else:
                print(' ---- Warning ----', CONNECTED_CF_ADDRESSES[cf_index][0], 'connection setup failed')


def cf_disconnected_callback(link_uri):
    global CONNECTED_CF_ADDRESSES
    global CF_READY

    for cf_index in range(len(CONNECTED_CF_ADDRESSES)):
        if CONNECTED_CF_ADDRESSES[cf_index][1] == link_uri:
            CF_READY[cf_index] = False
            print(CONNECTED_CF_ADDRESSES[cf_index][0], 'disconnected')


async def keyboard_handler():
    global CONNECTED_CF
    global WAYPOINT_NUMBERS
    global WAYPOINTS
    global SEQUENCE_ACTIVATED

    key_queue = detect_keyboard_input()
    while True:
        key = await key_queue.get()
        if key == pynput.keyboard.Key.esc:
            print(' ---- Warning ---- Emergency stop requested by user : disconnecting...')
            for cfs in CONNECTED_CF:
                cfs.commander.send_stop_setpoint()
            asyncio.get_event_loop().stop()
            print(' ---- Warning ---- Emergency stop requested by user : program killed')
        if key == pynput.keyboard.Key.up:
            print('  -> Takeoff')
            for cf_index in range(len(WAYPOINTS)):
                if WAYPOINT_NUMBERS[cf_index] is not None:
                    # noinspection PyTypeChecker
                    WAYPOINT_NUMBERS[cf_index] = 1
        if key == pynput.keyboard.Key.shift:
            print('  -> Flight sequence started')
            SEQUENCE_ACTIVATED = True
        if key == pynput.keyboard.Key.ctrl:
            print('  -> Flight sequence stopped, CFs hold their positions at the next reached waypoint')
            SEQUENCE_ACTIVATED = False
        if key == pynput.keyboard.Key.down:
            print('  -> Land')
            for cf_index in range(len(WAYPOINT_NUMBERS)):
                if WAYPOINT_NUMBERS[cf_index] is not None:
                    # noinspection PyTypeChecker
                    WAYPOINT_NUMBERS[cf_index] = len(WAYPOINTS[cf_index]) - 1


def detect_keyboard_input():
    queue = asyncio.Queue()
    loop = asyncio.get_event_loop()

    def on_press_callback(key):
        try:
            loop.call_soon_threadsafe(queue.put_nowait, key.char)
        except AttributeError:
            loop.call_soon_threadsafe(queue.put_nowait, key)

    pynput.keyboard.Listener(on_press=on_press_callback).start()
    return queue


def packet_reception_callback(packet: qtm.packet.QRTPacket):
    """ Callback : method called each time a packet is received from QTM """
    global QTM_BODIES
    global BODIES_NAMES
    global CF_FOUND
    global CONNECTED_CF
    global INVALID_6DOF_COUNT
    global WAYPOINT_NUMBERS
    global WAYPOINTS
    global SYNCHRONIZE_WAYPOINTS
    global ON_WAYPOINT
    global RUN_TRACKER
    global CF_READY
    global PACKET_COUNT
    global SEQUENCE_ACTIVATED

    #         -------- Security checks                 --------         #

    # The packet reception callback may contain a lot of instructions, and its execution time might be a problem.
    # When the callback execution time is too long, it cannot be run until the end before a new packet arrives and
    # interrupts its execution, resulting in instruction losses and unpredictable behaviours.
    # To avoid this issue, a tracker is implemented to make sure that, each time a new packet is received, the previous
    # packet callback has been executed until the end. If not, the flight sequence is stopped for every UAV.
    # Most common reasons for this issue to happen :
    #       -> A silent error somewhere which blocks program execution without interrupting it
    #       -> Too much computational load
    #               \_ Try to optimize the program
    #               \_ Try to decrease the packet emission rate on QTM
    if not RUN_TRACKER:
        print(' ---- Warning ---- Callback execution interrupted by new QTM packet')
        print('                   -> There might be an error occurring during callback execution')
        print('                   -> Or, the sequence might require too much computing load')
        for cf_to_stop in CONNECTED_CF:
            cf_to_stop.commander.send_stop_setpoint()
        WAYPOINT_NUMBERS = [None] * len(QTM_BODIES)

    RUN_TRACKER = False

    # Security check during multiple UAVs synchronized flight :
    # Makes sure that all in-flight UAVs are targeting the same waypoint number in their respective sequence
    # When a synchronization issue is detected, every UAV waypoint number is checked and/or changed to match the others
    if SYNCHRONIZE_WAYPOINTS:
        valid_wp_numbers = []
        valid_wp_numbers_index = []
        for i in range(len(WAYPOINT_NUMBERS)):
            if WAYPOINT_NUMBERS[i] is not None:
                valid_wp_numbers.append(WAYPOINT_NUMBERS[i])
                valid_wp_numbers_index.append(i)
        if len(valid_wp_numbers) > 0:
            wp_number_ref = valid_wp_numbers[0]
            if any(wp_number != wp_number_ref for wp_number in valid_wp_numbers):
                print(' ---- Warning ---- Synchronization error, resynchronizing...')
                for i in valid_wp_numbers_index:
                    # noinspection PyTypeChecker
                    WAYPOINT_NUMBERS[i] = max(valid_wp_numbers)
                # noinspection PyTypeChecker
                print('                   -> All in-flight UAVs moving towards their respective waypoint',
                      max(valid_wp_numbers))

        # Checks if all UAVs have reached their respective waypoint
        # If so, they can move towards the next waypoint
        # If not, they wait for all UAVs to reach their current waypoint
        all_on_waypoint = True
        for i in valid_wp_numbers_index:
            if not ON_WAYPOINT[i]:
                all_on_waypoint = False
        if all_on_waypoint:
            print('All in-flight UAVs on waypoint')
    else:
        all_on_waypoint = False

    #         -------- Data extraction and UAV control --------         #

    QTM_BODIES = qtm_tools.extract_packet_data(packet, QTM_BODIES, BODIES_NAMES)
    connected_cf_names = [cf_address[0] for cf_address in CF_FOUND]
    for uav_num in range(len(QTM_BODIES)):
        body = QTM_BODIES[uav_num]
        wp_number = WAYPOINT_NUMBERS[uav_num]
        if wp_number is not None:
            if body.valid_6dof:
                try:
                    cf_index = connected_cf_names.index(body.body_name)
                    if all(cf_ok for cf_ok in CF_READY):
                        CONNECTED_CF[cf_index].extpos.send_extpose(body.x, body.y, body.z,
                                                                   body.qx, body.qy, body.qz, body.qw)
                        INVALID_6DOF_COUNT[uav_num] = 0
                        if PACKET_COUNT[cf_index] > 10:
                            PACKET_COUNT[cf_index] = 0
                            send_packet = True
                        else:
                            PACKET_COUNT[cf_index] = PACKET_COUNT[cf_index] + 1
                            send_packet = False

                        wp_number, waypoints, ON_WAYPOINT = flight_sequence.waypoint_manager(body,
                                                                                             CONNECTED_CF[cf_index],
                                                                                             WAYPOINT_NUMBERS[uav_num],
                                                                                             uav_num,
                                                                                             WAYPOINTS[uav_num],
                                                                                             SYNCHRONIZE_WAYPOINTS,
                                                                                             all_on_waypoint,
                                                                                             send_packet,
                                                                                             ON_WAYPOINT,
                                                                                             SEQUENCE_ACTIVATED)
                        WAYPOINTS[uav_num] = waypoints
                    else:
                        CONNECTED_CF[cf_index].commander.send_stop_setpoint()
                except ValueError:
                    pass
            else:
                INVALID_6DOF_COUNT[uav_num] = INVALID_6DOF_COUNT[uav_num] + 1
                if INVALID_6DOF_COUNT[uav_num] > 10:
                    print(body.body_name, ': off camera for too long, switching the engines off')
                    wp_number = None
            WAYPOINT_NUMBERS[uav_num] = wp_number
        else:
            try:
                cf_index = connected_cf_names.index(body.body_name)
                CONNECTED_CF[cf_index].commander.send_stop_setpoint()
            except ValueError:
                pass

    if all_on_waypoint:
        ON_WAYPOINT = [False] * len(QTM_BODIES)

    if all(waypoint_number is None for waypoint_number in WAYPOINT_NUMBERS):
        print('All UAV tasks cleared, disconnecting...')
        asyncio.get_event_loop().stop()

    RUN_TRACKER = True


if __name__ == '__main__':
    #         -------- Waiting time before takeoff     --------         #

    # time.sleep(10)

    #         -------- Flight parameters               --------         #

    # QTM server IP address
    qtm_ip_address: str = '192.168.0.1'

    # Association array between QTM body names and Crazyflies radio identification numbers
    # [[1st cf name on QTM, 1st cf radio, 1st cf uri address],
    #  [2nd cf name on QTM, 2nd cf radio, 2nd cf uri address],
    #                     :
    #  [nth cf name on QTM, nth cf radio, nth cf uri address]]
    cf_addresses: List[List[str]] = [['cf1', '0', 'E7E7E7E701'],
                                     ['cf2', '0', 'E7E7E7E702'],
                                     ['cf3', '1', 'E7E7E7E703'],
                                     ['cf4', '1', 'E7E7E7E704']]

    WAYPOINTS = [[[None, None, None],  # cf1
                  [None, None, 1],  # Takeoff
                  [0.50, 0.50, 1],  # Beginning flight sequence
                  [1.00, 0.00, 1],
                  [0.75, -0.75, 1],
                  [-0.50, -0.50, 1],
                  [0.50, -0.50, 1],
                  [0.75, 0.00, 1],
                  [0.75, 0.00, 1],
                  [0.5, 0.5, 1],  # End of flight sequence
                  [None, None, 0]],  # Land

                 [[None, None, None],  # cf2
                  [None, None, 1],  # Takeoff
                  [0.50, -0.50, 1],  # Beginning flight sequence
                  [0.50, -0.50, 1],
                  [0.25, -0.25, 1],
                  [0.25, -0.25, 1],
                  [0.25, -0.25, 1],
                  [0.00, -0.75, 1],
                  [0.00, -0.75, 1],
                  [0.50, -0.50, 1],  # End of flight sequence
                  [None, None, 0]],  # Land

                 [[None, None, None],  # cf3
                  [None, None, 1],  # Takeoff
                  [-0.50, 0.50, 1],  # Beginning flight sequence
                  [0.00, 0.00, 1],
                  [0.50, 0.50, 1],
                  [0.75, -0.75, 1],
                  [-0.50, -0.50, 1],
                  [-0.75, 0.00, 1],
                  [-0.75, 0.00, 1],
                  [-0.50, 0.50, 1],  # End of flight sequence
                  [None, None, 0]],  # Land

                 [[None, None, None],  # cf4
                  [None, None, 1],  # Takeoff
                  [-0.50, -0.50, 1],  # Beginning flight sequence
                  [-0.50, -0.50, 1],
                  [-0.50, -0.25, 1],
                  [0.50, 0.50, 1],
                  [0.75, -0.75, 1],
                  [0.00, 0.75, 1],
                  [0.00, 0.00, 1],
                  [-0.50, -0.50, 1],  # End of flight sequence
                  [None, None, 0]]]  # Land

    # Boolean : True to make sure that all UAVs reached their respective waypoint before they move to the next ones
    #           False to allow UAVs to reach all their waypoints without waiting for the others
    SYNCHRONIZE_WAYPOINTS = True

    #         -------- QTM connection                  --------         #

    # Connects to QTM at provided IP address, and retrieves the tracked bodies names and their positions
    QTM_CONNECTION: qtm.QRTConnection = asyncio.get_event_loop().run_until_complete(
        qtm_tools.connect_to_qtm(qtm_ip_address))
    QTM_BODIES, BODIES_NAMES = asyncio.get_event_loop().run_until_complete(qtm_tools.scan_qtm_bodies(QTM_CONNECTION))
    print('Bodies tracked by QTM :', BODIES_NAMES)

    #         -------- Global variables initialization --------         #

    RUN_TRACKER = True
    INVALID_6DOF_COUNT = [0] * len(QTM_BODIES)
    POSITION_GOAL = [[0.0, 0.0, 0.0]] * len(QTM_BODIES)
    WAYPOINT_NUMBERS = [None] * len(QTM_BODIES)
    ON_WAYPOINT = [False] * len(QTM_BODIES)
    SEQUENCE_ACTIVATED = False

    #         -------- Crazyflies connection            --------         #

    # Scans the available Crazyflies from the provided address list and connects to them
    CF_FOUND, links = cf_tools.scan_available_cf(cf_addresses, len(QTM_BODIES))
    CONNECTED_CF, CONNECTED_CF_ADDRESSES = cf_tools.connect_to_cf(CF_FOUND)
    print('Connected cf :', CONNECTED_CF_ADDRESSES)

    PACKET_COUNT = [0] * len(CONNECTED_CF)
    CF_READY = [False] * len(CONNECTED_CF)

    for cf_name in CONNECTED_CF_ADDRESSES:
        try:
            cf_name_index = BODIES_NAMES.index(cf_name[0])
            # noinspection PyTypeChecker
            WAYPOINT_NUMBERS[cf_name_index] = 0
        except ValueError:
            print(' ---- Warning ----', cf_name, ': Crazyflie 6DoF model not declared in QTM')

    print('Waypoint numbers :', WAYPOINT_NUMBERS)

    # When a radio link is opened towards a Crazyflie, an automatic setup sequence is launched.
    # When this sequence is finished, a callback (cf_connected_callback()) is triggered as soon as
    # the drone allows us to access and change its own parameters via the radio link.
    for cf in CONNECTED_CF:
        cf.connected.add_callback(cf_connected_callback)
        cf.disconnected.add_callback(cf_disconnected_callback)

    #         -------- Main loop                        --------         #

    # Creates an event loop which runs the methods start_qtm_streaming() and keyboard_handler() as long as
    # the loop is not interrupted
    asyncio.ensure_future(start_qtm_streaming(QTM_CONNECTION))
    asyncio.ensure_future(keyboard_handler())
    asyncio.get_event_loop().run_forever()

    #         -------- End of the program               --------         #

    # Disconnects the Crazyflies, stops the QTM stream and disconnects from QTM
    cf_tools.disconnect_cf(CONNECTED_CF)
    asyncio.get_event_loop().run_until_complete(qtm_tools.disconnect_qtm(QTM_CONNECTION))
