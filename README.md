# Simple AirSim
This is a project for simplifying the use of the AirSim drone simulator.

### Airsim
[Airsim](https://github.com/microsoft/AirSim) is a simulator for drones, cars and more, built on [Unreal Engine](https://www.unrealengine.com/).
It is open-source, cross platform, and supports hardware-in-loop with popular flight controllers such as PX4 for physically and visually realistic simulations.
It is developed as an Unreal plugin that can simply be dropped into any Unreal environment.
Airsim's goal is to develop AirSim as a platform for AI research to experiment with deep learning, computer vision and reinforcement learning algorithms for autonomous vehicles.
For this purpose, AirSim also exposes APIs to retrieve data and control vehicles in a platform independent way.

But, AirSim is hard to work with. That's why we made this package.


## Getting Started
### the algo 
we created algorithm to make the drone fly himself.
we uesed main while loop for simpele Actions by using the API.
### the main loop 
```
def nav_algo(drone: Drone):
    # global TURNING_LEFT, TURNING_RIGHT
    # Drone's State Machine:
    # 1. Emergency
    # 2. Fix right wall
    # 3. Scan surroundings
    # 4. Move to target point
    # 5. Return home

    global sim_time  # = gettime()
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
    # yaw_pid = YawRatePID()
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
```

### the main loop works on 5 cases:
           
1) emergency: cheak if we too close to the wall around the drone.
2) battery: cheak if the battery is too low.
3) front < front_threshold: cheak if the front is go to a wall 
4) right > right_far_threshold: cheak if the drone is go out from the right wall 
5) fly_forward: if all is works, the drone go forward.
            
to go forward and not to crash we Calculating tan-1(right lidar/front lidar) and that gave us the angle to be parallel to the wall.
we used PID for that and for the move forward.


![Screenshot 2022-05-08 at 00 34 56](https://user-images.githubusercontent.com/66851296/167272377-f487109a-4c64-4aa8-b96f-2de5d9a9eeb2.png)


to know were we hane been, we used func "explore" in every time we in crossroads.
in "explore" we spin around and cheak with the lidars if there is obstacle, if not we keep the start place until we see obstacle,
then we add to the node:
1) position
2) visited, start and end vector to all the obstacles
and add to the graph.

![Screenshot 2022-05-08 at 00 49 35](https://user-images.githubusercontent.com/66851296/167272984-05cd7572-fc73-440c-88e0-c4dc521f60d0.png)











