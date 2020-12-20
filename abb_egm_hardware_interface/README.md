# abb_egm_hardware_interface

**Please note that this package has not been productized, and that academia is the intended audience.**\
**The package is provided "as-is", and as such no more than limited support can be expected.**

## Overview

The `abb_egm_hardware_interface` package provides:

- A `ros_control`-based hardware interface, for communicating with ABB robots via the *Externally Guided Motion* (`EGM`) interface:
  - Recommendations:
    - Use `RobotStudio` simulations before testing with real robots.
    - Use the latest supported `RobotWare` version to get the best `EGM` performance.
  - The hardware interface sets up `EGM` servers that the robot can send feedback messages to, and receive motion commands from:
    - It is always the robot that initiates `EGM` communication sessions.
  - The hardware interface will only allow `ros_control` controllers to be started if there is already an active `EGM` communication session:
    - Users can specify a list of controllers that are always ok to start (*e.g. passive state controllers*).
  - For `MultiMove` systems it is important to note that:
    - The ROS side configurations must specify correct names for the mechanical unit groups that are intended to be used with `EGM`.
    - Activation/deactivation of mechanical units, after the hardware interface has been initialized, is not handled by the hardware interface and can lead to unexpected behavior.
- A ROS node that uses the hardware interface, together with a `ros_control` controller manager, to run a control loop.
- A ROS node for stopping `ros_control` controllers when `EGM` sessions ends:
  - Users can specify a list of controllers that are ok to keep running (*e.g. passive state controllers*).

**Please note that this package is only recommended for advanced users.**

Please see the [egm_hardware_interface.launch](launch/egm_hardware_interface.launch) file for how to start the node and available node parameters. Additionally, the ROS node, provided by the `abb_rws_service_provider` package, is required during initialization of the hardware interface.

Please see the [config](config) folder for configuration examples for a few different use cases.

### Requirements

- `RobotWare` version `6.07.01` or higher (less than `7.0`).

Please see the underlying [abb_libegm](https://github.com/ros-industrial/abb_libegm) and [abb_librws](https://github.com/ros-industrial/abb_librws) packages for more details.

### Troubleshooting

*The following table is non-exhaustive, but it should at least give some ideas for things to check.*

| Issue | Check |
|  --- | --- |
| Communication | If the `FlexPendant` shows the `No data from UdpUc device` message it means that the robot sent out `EGM` `UDP` messages, but it did not receive any reply:<br><ul><li>Verify that the correct remote IP-address, and port number, has been specified in the the robot's system configurations: *`RobotStudio` > Controller tab > Configuration > Communication > Transmission Protocol* (check the `UDPUC` instances).</li><li>Verify that no firewall is blocking `UDP` communication on the desired ports.</li></ul> |
| Utilization rate | If the hardware interface prints messages about an `EGM` channel's utilization rate being above `100%`, it means that the desired commands sent by the user are too aggressive and they should be reduced. |
| Motion (*robot side settings*) | System configurations:<ul><li>See *`RobotStudio` > Controller tab > Configuration > Motion > External Motion Interface Data* (the `Filtering`/`Raw` level can for example affect how well references are tracked).</li></ul>Important `EGM` `RAPID` parameters:<ul><li>The `\MaxSpeedDeviation` parameter can be used to trim acceleration/deceleration, and it also seems to limit the maximum allowed joint speed (*ROS side velocity limits should be less than or equal to this this parameter*).</li><li>Pure velocity control requires the `\PosCorrGain` parameter to be set to `0`.</li><li>Make sure that the correct `tooldata` has be specified in the `RAPID` code with either `\Tool` or `\TLoad` (*i.e. if modal payload mode is set to `No` in the system configurations*).</li><li>If the `RobotWare` [StateMachine Add-In](https://robotapps.robotstudio.com/#/viewApp/c163de01-792e-4892-a290-37dbe050b6e1) is present, and if the ROS node provided by the `abb_rws_service_provider` package is used, then ROS services are exposed to get/set the `EGM` `RAPID` settings directly from the ROS side.</li></ul>*Please see the *Application manual - Externally Guided Motion* (*document ID: `3HAC073319-001`, revision: `B`*) for a more detailed description of `EGM` settings and capabilities.* |
| Motion (*ROS side settings*) | The hardware interface registers `PositionJointSoftLimitsInterface` and `VelocityJointSoftLimitsInterface` interfaces, which users can, for example, configure with velocity limits for each joint (*conservative limits are set per default*).<br><br>*Please see the [joint_limits_interface](https://github.com/ros-controls/ros_control/wiki/joint_limits_interface) package for more details about how joint limit configurations can be specified.*</li></ul></li></ul> |

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
