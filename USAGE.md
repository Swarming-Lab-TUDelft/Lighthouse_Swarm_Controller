# Usage

## Config file

Inside [config.py](ros2_ws/src/swarm_operation/swarm_operation/config.py) you can edit many parameters, e.g., example pattern, the number of drones, cage bounds, and collision avoidance.

At the top of the `config.py` file you can choose one of two pattern examples provided.
```python
#################### Example patterns ####################
POS_EXAMPLE, VEL_EXAMPLE = "PositionCommanderExample", "VelocityCommanderExample"
COMMANDER = POS_EXAMPLE
```
`POS_EXAMPLE` will organise the drones into a grid formation with a size depending on the number of drones in the swarm. `VEL_EXAMPLE` will result in an orbit like motion of the drones about a point above the origin.

Depending on your swarm setup, the number of drones and number of drones per radio have to be changed.

## Launching

If you are using **WSL**, first attach the crazyradios to WSL by running `collision_avoidance/WindowsBuild/attach_radios.exe`.


The package can be launched through `launch.py`:
```bash
ros2 launch swarm_operation launch.py
```

After running the launch command, start the collision avoidance. For **WSL** users, run `collision_avoidance/WindowsBuild/WindowsCA.exe`.

For **Linux** users, run `247Swarm/collision_avoidance/LinuxBuild/LinuxCA.x86_64`

Make sure to first launch the ROS package before the collision avoidance, such that the correct bounds are transfered.

## GUI
Through the GUI you can keep track of the current state of the drones and radios, control how many drones enter and exit the swarm, and switch between custom patterns you added.

![Screenshot of the GUI](images/GUI_screenshot.png)

In the top left, a scrollable list of all the drones will appear after a connection is established. Here you can see the state of the drone and the battery level. Clicking on one of these drone cards will open up a detailed information panel in the top center.

Same holds for the radio list on the bottom left.

On the right, you can see the swarm control panel. Here you can add and rmove drones to the swarm, see the number of drones in various states, and switch between custom patterns.

Whenever you want to close the application, **first** close the GUI. This will send a terminate command to all nodes. If some nodes are still running after this, use Ctrl+C in the terminal to shut them down. **Don't** use Ctrl+C in the terminal when the GUI is open, as this might result in a infinite shutdown loop.


## Custom swarming patterns/algorithms
All custom patterns are controlled by the "master_commander", as located in `swarm_operation/examples/`. This commander allows the live switching of different patterns. These patterns are listed in the `custom_swarm_commands` variable. If you want to create your own pattern, create a function that generates waypoints in `swarm_operation/examples/waypoint_functions.py`, and add a command in `master_commander.py` to the `custom_swarm_commands` dictionary. Also add on to the`MasterCommander` class in the same file - in `main_loop_cb`, so that the commander knows what to do when the command is received. Here you can call the function you've described in `waypoint_functions.py`.

Once added, a button should appear in the GUI automatically. This will allow you to activate the pattern.