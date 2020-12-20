# abb_rws_service_provider

**Please note that this package has not been productized, and that academia is the intended audience.**\
**The package is provided "as-is", and as such no more than limited support can be expected.**

## Overview

The `abb_rws_service_provider` package provides a ROS node that:

1. Attempts to connect to an ABB robot controller via the *Robot Web Services* (`RWS`) interface.
2. Collects information from the controller about the current system, for example:
   - `RobotWare` version and present options.
   - Configured mechanical unit groups (*e.g. robots and external axes*).
   - Configured `RAPID` tasks.
3. Exposes a set of ROS services for interacting with the controller from the ROS system.

Some of the exposed ROS services include:

- Basic interaction:
  - Setting motors on/off.
  - Starting/stopping `RAPID` execution.
  - Getting/setting of `RAPID` symbols (*e.g. constants and variables*):
    - Atomic `RAPID` data types: `bool`, `dnum`, `num` and `string`.
    - Complex `RAPID` data structures (*i.e. `RECORD`s, and these are currently only get/set via raw `RAPID` text format*).
  - Getting/setting of file contents (*of files in the robot controller's home directory*).
  - Getting/setting speed ratio for `RAPID` motions.
  - Getting/setting of IO-signals.
- `RobotWare` [StateMachine Add-In](https://robotapps.robotstudio.com/#/viewApp/c163de01-792e-4892-a290-37dbe050b6e1) interaction (*only exposed if the `StateMachine Add-In` is present in the system*):
  - `RAPID` interaction:
    - Setting custom `RAPID` routines to run (*the routines needs to be predefined in the `RAPID` code*).
    - Signaling that the custom `RAPID` routines should be run.
  - *Externally Guided Motion* (`EGM`) interaction (*only exposed if the `EGM` option is present*):
    - Getting/setting `EGM` `RAPID` settings.
    - Starting/stopping `EGM` joint or pose motions.
    - Starting/stopping `EGM` position streaming (*i.e. without controlling motions*).
  - `SmartGripper` interaction (*only exposed if the `SmartGripper` option is present*):
    - Setting commands to run (*e.g. grip in/out or turn on/off vacuum*).
    - Signaling that the commands should be run.

Please see the [rws_service_provider.launch](launch/rws_service_provider.launch) file for how to start the node and available node parameters. Additionally, the ROS node, provided by the `abb_rws_state_publisher` package, is required for most of the service calls.

### Requirements

- `RobotWare` version `6.07.01` or higher (less than `7.0`).

Please see the underlying [abb_libegm](https://github.com/ros-industrial/abb_libegm) and [abb_librws](https://github.com/ros-industrial/abb_librws) packages for more details.

### Troubleshooting

*The following table is non-exhaustive, but it should at least give some ideas for things to check.*

| Issue | Check |
|  --- | --- |
| Authentication | The node is currently hardcoded to use the default `RWS` login (`Default User` and `robotics`), but this can be changed in the [code](src/rws_service_provider.cpp#L106-L107) if needed. |
| Communication | Verify that the correct IP-address has been specified for the robot controller, and that it can be reached on the network:<br><ol><li>Using `ping <IP-address>`.</li><li>Using a web browser with `http://<IP-address>/rw?debug=1`, which will show a login prompt if connected (*e.g. use the default login above*).</li></ol> |
| `RobotStudio` | `RobotStudio` simulations blocks remote `RWS` connections by default, which is shown by the `RAPI Unidentified Error` message (*e.g. when using a web browser*).<br><br>*Please see this [post](https://forums.robotstudio.com/discussion/12082/using-robotwebservices-to-access-a-remote-virtual-controller) on the official `RobotStudio` forum for how to allow remote `RWS` connections.* |
| ROS Services | Verify that resource names are correctly spelled, for example:<br><ul><li>IO-signal name.</li><li>`RAPID` task, module and symbol names.</li><li>File name.</li><li>Etc.</li></ul>*Please note that most of the exposed services, that sets resources, requires auto mode to be active.* |

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
