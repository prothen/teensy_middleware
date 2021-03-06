# Teensy middleware
This is a convenience repository to alleviate setup overhead for similar projects using microcontrollers (uc) with ROS interfaces or similar serial based middleware software.

In this framework possibly private middleware message repositories are cloned through individially configured deploy keys to clone and build corresponding header files to be used in the uc firmware.

## Repository structure
- [Tests](firmware/tests.ini)
- [Target definition](firmware/targets)
- [Source code location](firmware/src/targets)
- [Target specification](firmware/platformio.ini)
- [Developer instructions](docs/DEVELOPMENT.md)

_Note: Recommended toolchain [`platformio`](https://docs.platformio.org/en/latest/what-is-platformio.html), [`scons`](https://www.scons.org/), `python3` and `chibiOS`._

## Usage (ROS1 example)
You can flash your firmware from any location simply by executing
```
flashme YourTarget
```
_Note: With a test target for example with `flashme test_ros`._


Alternatively, it is possible to flash from within the `firmware` folder using the platformio default
```bash
pio run -e YourTarget
```
_Note:_ This defaults to `pio run -e YourTarget -t upload`

You can find templates for how to setup your workspace with example environments and configurations under [`firmware/platformio.ini`](firmware/platformio.ini).


If you have a ROS1 distribution installed you can test the middleware generation for standard messages with `platformio run -e test_ros1`.

## Installation
Ensure that you have sourced your ROS1 distribution, then follow the subsequent instructions and re-login before moving on to the [usage](#usage) section.
Execute
```
./init.sh
```

_Note: This configures a environment variable `tmw_DIR` to the current working directory and allows using the hooks and shell scripts for generating middleware header files._

## FAQ

<details>
<summary>
The ROS header file cannot be found.
</summary>


Ensure that you ran `init.sh` (required to use automated hooks) and that you restarted your login session. (reboot or re-login)
You can test if `echo ${tmw_DIR}` outputs the directory of your `teensy_firmware` clone.

Also ensure that you have a `catkin` compatible Python version. (Note that Python 3.7 has a trollius and async issue. Easiest is to switch your virtualenvironment to Python2 and install the dependencies with `pip install -r requirements.txt`. Also ensure that after the virtual environment switch you should execute `source /opt/ros/${ROS_DISTRO}/setup.{sh,zsh}`).
</details>


<details>
<summary>
The build fails and throws some error.
</summary>


Inspect the logs generated by the hooks under `./firmware/log/middleware.log`


The output could look similar to the following snippet.
```bash
(vp2) firmware cat log/middleware.log
Input arguments received:
	 template git clone https://github.com/prothen/testbed_msgs.git
Set project name: template
Parse repository:
	 git clone https://github.com/prothen/testbed_msgs.git
Received additional libraries.
ROS1 interface chosen. Configuring dependencies ... (tzz its 2020...)
BUILD rosserial arduino from upstream:
remote: Enumerating objects: 467, done.
remote: Counting objects: 100% (467/467), done.
remote: Compressing objects: 100% (351/351), done.
remote: Total 467 (delta 115), reused 297 (delta 60), pack-reused 0
Receiving objects: 100% (467/467), 299.88 KiB | 3.57 MiB/s, done.
Resolving deltas: 100% (115/115), done.
----------------------------------------------------------------
```
</details>

## Contribution
Any contribution is welcome.
If you find missing instructions or something did not work as expected please create an issue and let me know.

## License
See the `LICENSE` file for details of the available open source licensing.
