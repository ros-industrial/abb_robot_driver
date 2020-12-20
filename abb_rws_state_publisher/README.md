# abb_rws_state_publisher

**Please note that this package has not been productized, and that academia is the intended audience.**\
**The package is provided "as-is", and as such no more than limited support can be expected.**

## Overview

The `abb_rws_state_publisher` package provides a ROS node that:

1. Attempts to connect to an ABB robot controller via the *Robot Web Services* (`RWS`) interface.
2. Collects information from the controller about the current system, for example:
   - `RobotWare` version and present options.
   - Configured mechanical unit groups (*e.g. robots and external axes*).
   - Configured `RAPID` tasks.
3. Starts to periodically poll the controller for runtime data, parse the collected data into ROS messages and publish them to the ROS system.

Some of the data published to the ROS system include:

- General system states:
  - If motors are on/off.
  - If auto/manual operation mode is active.
  - If `RAPID` execution is running/stopped.
  - State of each `RAPID` task.
  - State of each mechanical unit.
- Joint positions of each mechanical unit.
- Runtime state of each `RAPID` program instance of the `RobotWare` [StateMachine Add-In](https://robotapps.robotstudio.com/#/viewApp/c163de01-792e-4892-a290-37dbe050b6e1):
  - *Only collected and published if the `StateMachine Add-In` is present in the system*.

Please see the [rws_state_publisher.launch](launch/rws_state_publisher.launch) file for how to start the node and available node parameters.

### Requirements

- `RobotWare` version `6.07.01` or higher (less than `7.0`).

Please see the underlying [abb_libegm](https://github.com/ros-industrial/abb_libegm) and [abb_librws](https://github.com/ros-industrial/abb_librws) packages for more details.

### Troubleshooting

*The following table is non-exhaustive, but it should at least give some ideas for things to check.*

| Issue | Check |
|  --- | --- |
| Authentication | The node is currently hardcoded to use the default `RWS` login (`Default User` and `robotics`), but this can be changed in the [code](src/rws_state_publisher.cpp#L122-L123) if needed. |
| Communication | Verify that the correct IP-address has been specified for the robot controller, and that it can be reached on the network:<br><ol><li>Using `ping <IP-address>`.</li><li>Using a web browser with `http://<IP-address>/rw?debug=1`, which will show a login prompt if connected (*e.g. use the default login above*).</li></ol> |
| `RobotStudio` | `RobotStudio` simulations blocks remote `RWS` connections by default, which is shown by the `RAPI Unidentified Error` message (*e.g. when using a web browser*).<br><br>*Please see this [post](https://forums.robotstudio.com/discussion/12082/using-robotwebservices-to-access-a-remote-virtual-controller) on the official `RobotStudio` forum for how to allow remote `RWS` connections.* |

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
