# Autonomous Drone Simulation
This is a project for simplifying the use of the AirSim drone simulator.

### Airsim
[Airsim](https://github.com/microsoft/AirSim) is a simulator for drones, cars and more, built on [Unreal Engine](https://www.unrealengine.com/).
It is open-source, cross platform, and supports hardware-in-loop with popular flight controllers such as PX4 for physically and visually realistic simulations.
It is developed as an Unreal plugin that can simply be dropped into any Unreal environment.
Airsim's goal is to develop AirSim as a platform for AI research to experiment with deep learning, computer vision and reinforcement learning algorithms for autonomous vehicles.
For this purpose, AirSim also exposes APIs to retrieve data and control vehicles in a platform independent way.


## 
### The Simulation
Our goal is to make the simulated drone fly in a safe manner around any given map using 
a Navigation algorithm, PID Controllers (For pitch and yaw angels) and Obstacle avoidance logic.
### Drone's Control Loop:

```
def nav_algo(drone: Drone):

    # Takeoff
    drone.command(0, 0, 0, HEIGHT_Z_AXIS, True)
    
    # Add starting point to the graph:
    graph.append((drone.get_position()['x'],
                  drone.get_position()['y'],
                  drone.get_position()['z']))
                  
    start_time = time.time()
   
    # Loop starts here and never ends
    while True:
    
        # Gather all surroundings information from the drone's lidars 
        lidars = drone.get_lidars()
        position = drone.get_position()
        orientation = drone.get_orientation()
        velocity = drone.get_velocity()

        front = lidars['front']
        right = lidars['right']
        left = lidars['left']

        # Our drone's `State machine`:
        
        if front < emergency_threshold:
            print("Emergency!")
            emergency(drone)
            
        elif front < front_threshold:
            print("front < front_threshold!")
            # explore(drone)
            # Rotate CCW
            rotate_ccw(drone)

        elif right > right_far_threshold:
            print("right > right_far_threshold!")
            # Rotate CW
            drone.command(0, 0, -60, HEIGHT_Z_AXIS, False)
            
        elif battery_low:
            return_home(drone)
            # Handle emergency actions:
            
        else:
            # Do mission (Explore the area)
            fly_forward(drone)
```

### Drone's State Machine:
           
1) Emergency: When front lidar indicates `objcect is close`, the drone will stop and figure out its surroundings.
2) Battery: Chcek battery status - if low: the drone will return to its home point.
3) Front < front_threshold: If drone's front sight is limited by an object (wall), Rotate the drone CCW to avoide.
4) Right > right_far_threshold: Check if the drone's right lidar shows gra
5) Fly_forward: if all other states are not active - the drone wants to explore the area
            
to go forward and not to crash we Calculating tan-1(right lidar/front lidar) and that gave us the angle to be parallel to the wall.
we used PID for that and for the move forward.

### PID - Proportional, Integral and Derivitive errors:
For a "cleaner" flight, we've implemented a PID Controller.
A PID Controller calculates the drone's desired angles for movement

![Screenshot 2022-05-08 at 00 34 56](https://user-images.githubusercontent.com/66851296/167272377-f487109a-4c64-4aa8-b96f-2de5d9a9eeb2.png)

To know where we have been, we used func "explore" in every time we in crossroads.
in "explore" we spin around and cheak with the lidars if there is obstacle, if not we keep the start place until we see obstacle,
then we add to the node:
1) Drone's Position [x, y, z]
2) Start and end vector to all the obstacles
3) Visited or not.

![Screenshot 2022-05-08 at 00 49 35](https://user-images.githubusercontent.com/66851296/167272984-05cd7572-fc73-440c-88e0-c4dc521f60d0.png)

## More:

### Takeoff
![airsim](https://user-images.githubusercontent.com/63110245/167283908-88d859c5-c353-4441-adb7-fe811b0fafe2.png)










