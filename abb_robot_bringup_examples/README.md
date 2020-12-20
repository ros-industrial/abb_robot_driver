# abb_robot_bringup_examples

[![license - bsd 3 clause](https://img.shields.io/:license-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![support level: vendor](https://img.shields.io/badge/support%20level-vendor-brightgreen.svg)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

**Please note that this package has not been productized, and that academia is the intended audience.**\
**The package is provided "as-is", and as such no more than limited support can be expected.**

## Overview

The ROS nodes included in [abb_robot_driver](https://github.com/ros-industrial/abb_robot_driver) can only interact with ABB robots that are supported by these interfaces:

- *Robot Web Services* (`RWS`) `1.0`:
  - Available in `IRC5` controllers with `RobotWare 6` systems.
  - `RWS` is independent of robot type.
- *Externally Guided Motion* (`EGM`):
  - Requires the `RobotWare 6` option *Externally Guided Motion* (`689-1`).
  - `EGM` is only available for `6` and `7` axes robots.

The following examples illustrates how to bringup the ROS nodes for a few different use cases, namely:

| Example | Description |
| --- | --- |
| [ex1_rws_only](#example-1-ex1_rws_only) | Using only `RWS` components, e.g. to listen for general system states and restarting `RAPID` program execution. |
| [ex2_rws_and_egm_6axis_robot](#example-2-ex2_rws_and_egm_6axis_robot) | Using both `RWS` and `EGM` components (*with a 6-axes robot*), i.a. for starting an `EGM` session and publishing joint velocity commands. |
| [ex3_rws_and_egm_yumi_robot](#example-3-ex3_rws_and_egm_yumi_robot) | Using both `RWS` and `EGM` components (*with a `YuMi` robot*), i.a. for starting an `EGM` session and publishing joint velocity commands. |

Please inspect the files in the [launch](launch) and [config](config) folders for more details about respective example.

## Examples

**Please note that it is recommended to try out the examples against `RobotStudio` simulations before testing with real robots.**

### Example 1: "ex1_rws_only"

- The `RWS` components are independent of robot type.

#### Steps

1. Start 3 terminals.
2. **[Terminal 1]** Launch the example:
   ```
   roslaunch abb_robot_bringup_examples ex1_rws_only.launch robot_ip:=<robot controller's IP address>
   ```
3. **[Terminal 2]** Use `rostopic` to, for example, list available topics and then listen for general robot controller states:
   ```
   rostopic list
   rostopic echo -c /rws/system_states
   ```
4. **[Terminal 3]** Use `rosservice` to, for example, list available services and then stop, reset and start the `RAPID` execution:
   ```
   rosservice list
   rosservice call /rws/stop_rapid "{}"
   rosservice call /rws/pp_to_main "{}"
   rosservice call /rws/start_rapid "{}"
   ```

---

### Example 2: "ex2_rws_and_egm_6axis_robot"

- **`EGM` usage is only recommended for advanced users:**
  - **Please be careful when sending motion commands! The robot will respond immediately.**
- Assumptions:
  - A 6-axes robot is used.
  - The `EGM` `RobotWare` option is present in the robot controller system.
  - The `RobotWare` [StateMachine Add-In](https://robotapps.robotstudio.com/#/viewApp/c163de01-792e-4892-a290-37dbe050b6e1) was used when installing the robot controller system.
- Impactful `EGM` `RAPID` arguments:
  - The `\MaxSpeedDeviation` argument of `EGMActJoint` (*e.g. limits `EGM` references on the robot controller side*).
  - The `\PosCorrGain` argument of `EGMRunJoint` (*e.g. needs to be `0` for pure velocity control*).
- The `ros_control` controller, which command motions, is only allowed to start if an `EGM` session is active.

#### Steps

1. Start 4 terminals.
2. **[Terminal 1]** Launch the example:
   ```
   roslaunch abb_robot_bringup_examples ex2_rws_and_egm_6axis_robot.launch robot_ip:=<robot controller's IP address>
   ```
3. **[Terminal 2]** Use `rostopic` to listen for `EGM` channel states:
   ```
   rostopic echo -c /egm/egm_states
   ```
4. **[Terminal 3]** Use `rosservice` to restart `RAPID`, and then start an `EGM` session:
   ```
   rosservice call /rws/stop_rapid "{}"
   rosservice call /rws/pp_to_main "{}"
   rosservice call /rws/start_rapid "{}"
   rosservice call /rws/sm_addin/start_egm_joint "{}"
   ```
5. **[Terminal 3]** Use `rosservice` to start the `ros_control` command controller (*requires a running `EGM` session*):
   ```
   rosservice call /egm/controller_manager/switch_controller "start_controllers: [joint_group_velocity_controller]
   stop_controllers: ['']
   strictness: 1
   start_asap: false
   timeout: 0.0"
   ```
6. **[Terminal 4]** Use `rostopic` to publish velocity commands for the sixth joint:
   ```
   rostopic pub /egm/joint_group_velocity_controller/command std_msgs/Float64MultiArray "data: [0,0,0,0,0,0.1]"
   ```
7. **[Terminal 3]** Use `rosservice` to stop the `EGM` session:
   ```
   rosservice call /rws/sm_addin/stop_egm "{}"
   ```

---

### Example 3: "ex3_rws_and_egm_yumi_robot"

- **`EGM` usage is only recommended for advanced users:**
  - **Please be careful when sending motion commands! The robot will respond immediately.**
- Assumptions:
  - A `YuMi` robot is used.
  - The `EGM` `RobotWare` option is present in the robot controller system.
  - The `RobotWare` [StateMachine Add-In](https://robotapps.robotstudio.com/#/viewApp/c163de01-792e-4892-a290-37dbe050b6e1) was used when installing the robot controller system.
- Impactful `EGM` `RAPID` arguments:
  - The `\MaxSpeedDeviation` argument of `EGMActJoint` (*e.g. limits `EGM` references on the robot controller side*).
  - The `\PosCorrGain` argument of `EGMRunJoint` (*e.g. needs to be `0` for pure velocity control*).
- The `ros_control` controller, which command motions, is only allowed to start if an `EGM` session is active.
- The ROS nodes are launched in an separate namespace `/yumi` (*i.e. it is good practice to do so*):
  - Inspect the example launch file, and corresponding configurations files, to see the impact of a separate namespace.

#### Steps

1. Start 4 terminals.
2. **[Terminal 1]** Launch the example:
   ```
   roslaunch abb_robot_bringup_examples ex3_rws_and_egm_yumi_robot.launch robot_ip:=<robot controller's IP address>
   ```
3. **[Terminal 2]** Use `rostopic` to listen for `EGM` channel states:
   ```
   rostopic echo -c /yumi/egm/egm_states
   ```
4. **[Terminal 3]** Use `rosservice` to restart `RAPID`, and then start an `EGM` session:
   ```
   rosservice call /yumi/rws/stop_rapid "{}"
   rosservice call /yumi/rws/pp_to_main "{}"
   rosservice call /yumi/rws/start_rapid "{}"
   rosservice call /yumi/rws/sm_addin/start_egm_joint "{}"
   ```
5. **[Terminal 3]** Use `rosservice` to start the `ros_control` command controller (*requires a running `EGM` session*):
   ```
   rosservice call /yumi/egm/controller_manager/switch_controller "start_controllers: [joint_group_velocity_controller]
   stop_controllers: ['']
   strictness: 1
   start_asap: false
   timeout: 0.0"
   ```
6. **[Terminal 4]** Use `rostopic` to publish velocity commands for the seventh joint (on each arm):
   ```
   rostopic pub /yumi/egm/joint_group_velocity_controller/command std_msgs/Float64MultiArray "data: [0,0,0,0,0,0,-0.1,0,0,0,0,0,0,0.1]"
   ```
7. **[Terminal 3]** Use `rosservice` to stop the `EGM` session:
   ```
   rosservice call /yumi/rws/sm_addin/stop_egm "{}"
   ```

## Acknowledgements

### ROSIN Project

<p>
  <a href="http://rosin-project.eu">
    <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" alt="rosin_logo" height="50" align="left">
  </a>
  The core development has been made within the European Union's Horizon 2020 project: ROSIN - ROS-Industrial Quality-Assured Robot Software Components (see http://rosin-project.eu for more info).
  <br><br>
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg" alt="eu_flag" height="50" align="left">
  The ROSIN project has received funding from the European Union's Horizon 2020 research and innovation programme under grant agreement no. 732287.
</p>

*The opinions expressed here reflects only the author's view and reflects in no way the European Commission's opinions. The European Commission is not responsible for any use that may be made of the contained information.*

### Special Thanks

Special thanks to [gavanderhoorn](https://github.com/gavanderhoorn) for guidance with open-source practices and conventions.
