import time

from FrontPID import FrontPID
from math import *

# from YawRatePID import YawRatePID
from simple_airsim.api import coordinate_system
from simple_airsim.api.drone import Drone
from simple_airsim.api.gui_manager import GUIManager
from simple_airsim.api.manager import Manager

# ==== GLOBAL CONSTANTS ====
sim_time = 0
emergency_threshold = 0.3
front_threshold = 0.8
right_far_threshold = 0.4
tunnel_threshold = 0.25
HEIGHT_Z_AXIS = -1.025
PRECISION = 0.00001
INF_DISTANCE = 3.0
FLIGHT_TIME_MIN = 8
FLIGHT_SPEED = 0.1
LIDAR_ERR = 0.02
# ===========================

# ==== Drone's States ====
TURNING_RIGHT = False
TURNING_LEFT = False
EXPLORING = False
EMERGENCY = False
# ========================


# ===== GLOBAL VARIABLES =====
graph = []  # Drone's waypoints - list of nodes (Waypoints)
# ============================

# === Drone's PID Controllers ===
# roll_pid = RightWallPID(target=right_far_threshold)
pitch_front_pid = FrontPID(target=front_threshold)  # 0.8  =  0.7 + 0.1
# yaw_pid = YawRatePID(target=0.0)
# ===============================


# לוקח מיקום הזווית פנים ואז בלולאת פור עובר על כל מעלה ובודק אם יש משהו מולו ואם לא הוא שומר את ההתחלה של ה״איזור בטוח״ עד מפגש עם משהו מולו ואז שומר את הסוף שך האיזור הבטוח, כך נשצור את המקומות האפשריים למעבר ונכניס לגרף
def explore(drone: Drone):
    global graph
    node = []
    temp_l = drone.get_lidars()
    orientation = drone.get_orientation()
    position = drone.get_position()
    velocity = drone.get_velocity()
    visited = False
    node.append(position)
    start_front = orientation['pitch']  # להוסיף גם את מאיפה הגענו ולסמת שביקרנו בו
    end_front = 0
    flag_front = True

    alpha = drone.get_orientation()['yaw']
    goal = alpha + 85
    while alpha < goal:
        alpha = drone.get_orientation()['yaw']
        drone.command(roll=0, pitch=0, yaw_rate=-30, z=HEIGHT_Z_AXIS, wait=False)
        if temp_l['front'] < INF_DISTANCE and flag_front == True:
            flag_front = False
            end_front = orientation['pitch']
            node.append((visited, start_front, end_front))
        if temp_l['front'] >= INF_DISTANCE and flag_front == False:
            flag_front = True
            start_front = orientation['pitch']
        temp_l = drone.get_lidars()

    graph.append(node)
    drone.command(0, 0, 0, HEIGHT_Z_AXIS, True)  # Stop

    print(f'Node: {node}')


def emergency(drone: Drone):
    global pitch_front_pid
    print("Emergency mode!")
    # drone.hover(wait=True)
    pitch_front_pid = FrontPID(target=emergency_threshold)
    while True:
        n_front = drone.get_lidars()['front']
        n_right = drone.get_lidars()['right']
        # drone.move_by(x=-abs(front-0.5), y=-abs(right-0.5), z=0, wait=False)
        p = pitch_front_pid.compute(n_front)
        print(f'Pitch output: {p}')
        drone.command(0, p, 0, z=HEIGHT_Z_AXIS, wait=False)

        if abs(n_front - emergency_threshold) < 0.05 and abs(n_right - emergency_threshold) < 0.05:
            print(f'Out of danger!')
            break
    print("Out of emergency")


def return_home(drone: Drone):
    print("Returning home")
    print(f'Graph of (x,y,z) coordinates to go home: {graph}')
    for i in range(0, len(graph)):
        go_to = graph.pop(len(graph) - 1)[0]  # הנ״צ של הנוד האחרון (האם נמחק ה pop כל פעם?)
        # חישוב הווית של הרחפן לנקודה
        # נסיעה לשם
        drone.command(roll=0, pitch=0, yaw_rate=0, z=HEIGHT_Z_AXIS, wait=True)  # move to...

    pass


def get_move_vector(drone: Drone):
    print("get_move_vector home")
    node = graph.pop()
    center = 0
    for i in range(0, len(node)):  # כל עוד לא עברתי על כל הנוד
        if node[i] == False:
            node[i] = True
            return center  # התחלה פחות סוף האיזור הבטוח חלקי 2) וכל זה נוסיף למבט שלי (צריך לראות ערכים מוחלטים וכו)
    # אם אין נקודה שבה לא היינו (כלומר הכל כבר true ) אז תחזור אחורה מאיפה שבאנו
    return 180  # return back


def fly_forward(drone: Drone):
    print("Flying forward")
    lidars = drone.get_lidars()
    vel = drone.get_velocity()

    # get lidars
    front = min(lidars['front'], INF_DISTANCE)
    right = min(lidars['right'], INF_DISTANCE)

    print(f'Front: {front} | Right: {right}')
    Vx = vel['x']
    yaw_rate = 0
    roll = 0

    if right < right_far_threshold + 0.3:
        yaw_rate = 20   # deg /sec

    pitch = pitch_front_pid.compute(front)
    # Bound the drone's speed
    if Vx >= FLIGHT_SPEED:
        pitch = 0
    drone.command(roll=roll, pitch=pitch, yaw_rate=yaw_rate, z=HEIGHT_Z_AXIS, wait=False)


def rotate_ccw(drone: Drone):
    front, right = (drone.get_lidars()['front'],
                    drone.get_lidars()['right'])
    alpha = degrees(atan(right/front))

    yaw_rate = yaw_pid.compute(alpha)
    # if right < right_far_threshold:
    # print(f'Right < right_far_threshold!!!')

    pitch = pitch_front_pid.compute(front)
    print(f'pitch = {pitch}')
    # if velocity['x'] >= 0.01:
    #     pitch = 0
    drone.command(0, pitch, yaw_rate, HEIGHT_Z_AXIS, False)


def nav_algo(drone: Drone):
    # global TURNING_LEFT, TURNING_RIGHT
    # Drone's State Machine:
    # 1. Emergency
    # 2. Fix right wall
    # 3. Scan surroundings
    # 4. Move to target point
    # 5. Return home

    global sim_time  # = gettime()
    # home point (node with vector)
    battery_low = False

    # Takeoff
    drone.command(0, 0, 0, HEIGHT_Z_AXIS, True)
    # Add starting point to the graph:
    graph.append((drone.get_position()['x'],
                  drone.get_position()['y'],
                  drone.get_position()['z']))
    print(f'Home point: {graph[0]}')
    i = 0
    time_last = 0
    start_time = time.time()
    time_sec_last = 0
    lidars = drone.get_lidars()
    yaw_pid = YawRatePID()
    # explore(drone)
    while True:
        lidars = drone.get_lidars()
        position = drone.get_position()
        orientation = drone.get_orientation()
        velocity = drone.get_velocity()

        sim_time = time_last - start_time
        time_sec = int(sim_time)
        if time_sec > time_sec_last:
            if i % 1000 == 0:
                print("Time now: ", float('%.1f' % sim_time))
        if time_sec >= 60 * FLIGHT_TIME_MIN / 2:
            battery_low = True
        time_sec_last = time_sec

        if i == 1000000:
            i = 0

        front = lidars['front']
        right = lidars['right']
        left = lidars['left']
        # down = lidars['down']
        # x = position['x']
        # y = position['y']
        # z = position['z']w
        # yaw_rate = yaw_pid.compute(front, right, target=???)

        # print(f'yaw_rate output: {yaw_rate}')
        if front < emergency_threshold:
            print("Emergency!")

        elif battery_low:
            return_home(drone)
            # Handle emergency actions:

            # emergency(drone)
        elif front < front_threshold:
            print("front < front_threshold!")
            # explore(drone)
            # Rotate CCW
            rotate_ccw(drone)

        elif right > right_far_threshold:
            print("right > right_far_threshold!")
            # Rotate CW
            drone.command(0, 0, -60, HEIGHT_Z_AXIS, False)

        else:
            # Do mission (Explore the area)
            fly_forward(drone)

        # print(f'Movement vector: [{roll}, {pitch}, {yaw}]')
        # drone.command(roll=roll, pitch=pitch, yaw_rate=yaw, z=HEIGHT_Z_AXIS, wait=False)
        i += 1
        time_last = time.time()


if __name__ == '__main__':
    with Manager(coordinate_system.AIRSIM, method=nav_algo) as man:
        with GUIManager(man, 10, 10, 10, 3) as gui:
            gui.start()

"""        if abs(front - emergency_threshold + 0.1) < PRECISION:
            # drone.command(roll=0.0, pitch=0.0, yaw_rate=0.0, z=HEIGHT_Z_AXIS, wait=False)
            pos = drone.get_position()
            # print(f"Drone achieved position ({pos['x']},{pos['y']},{pos['z']}) ")
            # EXPLORING = False
            # pitch = 0
            # roll = 0
            break"""
