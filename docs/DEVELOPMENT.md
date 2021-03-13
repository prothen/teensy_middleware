# Development Instructions


### Features
The teensy middleware framework provides build automation for middleware. This means that message dependencies are defined through a `yaml` file and build automatically.

The triggered hooks setup the `udev` rules and in the process you have to physically reconnect the device.

After the reboot the automatic boot loader invocation is triggered during the build process and no user intervention is required.

## Custom Target
Adding your own target is simple
- Create a `target directory` under `firmware/targets/YourTargetName/`
    - Add your directory to `firmware/platformio.ini`
    - Add `config.yaml` in your `target directory`  (Defines your middleware repositories)
    - _Note: See references for `PlatformIO` and `SCons` for usage and existing configurations in this repository._

### Middleware
The build environment supports automated generation of message libraries.
The message generation is defined in the `config.yaml` file accompanying the `platformio.ini` in the `firmware/targets/PROJECTNAME` directory.

The first build generates the necesseray libraries into `firmware/external` and will subsequently skip the rebuild if these directories exist.
If no additional ROS message libraries besides the standard messages are desired, the only required key in `config.yaml` is `PROJECTNAME: ros1:` (see e.g. [`firmware/targets/example/config.yaml`](../firmware/targets/example/config.yaml) without `example_msgs`).

_Note:_ The messages will *not* be re-downloaded and rebuilt unless the `firmware/external/PROJECTNAME` folder is first manualy deleted.

Currently there is only ROS1 support to generate middleware:
- Build automation then generates message headers for serialisation and deserialisation based on your configuration.
- In order to add automated build support for your own custom middleware message repositories it is recommend to create a lightweight message-only repository of the form `project_msgs`, which can be added to the `./targets/YourTargetName/config.yaml` with the tags `{ref,deploy_key}`:
    - `ref`: Is the ssh link to the repository
    - `deploy_key`: Is **only** necessary for private repositories. (see note below)

_Note: Optional for closed source repositories add the `deploy_key: keyname` (located in `./resources/deploy_keys/`).
Use `ssh-keygen`, then add the public key `*.pub` to the message repository and add the private key to the folder `resources/deploy_keys`.
The key needs to be generated without a password. The `.gitignore` is by default ignoring all keys in `resources/deploy_keys/`.
New keys need to be added with `git add -f resources/deploy_keys/YourNewKey`._

## Teensy Hardware Specs
- PJRC Teensy 4.0 ARM Cortex-M7 600MHz (NXP iMXRT1062 chip)
- 2MB Flash
- 1MB RAM (two banks)
    - RAM1 - 512KB : FASTRUN (ITCM, DTCM)
    - RAM2- 512KB : DMAMEM

## Abbreviations
- `ITCM`: instruction tightly coupled memory [link](https://www.kernel.org/doc/html/latest/arm/tcm.html) - no chache, access predictable and each cycle
- `DTCM`: data tightly coupled memory - see link above

## References
- ARM Cortex M7 Technical Reference [link](http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.ddi0489d/index.html)
- _tickless-mode_: HW counter of physical timer - no system interrupts [link](http://www.chibios.org/dokuwiki/doku.php?id=chibios:articles:tickless)
- _rosserial_arduino_ : getting started [link](https://wiki.ros.org/rosserial_arduino/Tutorials/Blink)(obsolete)
    - `rosrun rosserial_arduino serial_node.py /dev/ttyACM0` : connect from workstation / desktop to embedded device
- _SCons_ software construction build utility tool API [link](https://scons.org/doc/latest/HTML/scons-api/index.html)
- XRCE-DDS (NTP based time synchronization) [link](https://micro-xrce-dds.readthedocs.io/en/latest/time_sync.html)
- YAML 3rd standard `1.2` - [link](https://yaml.org/spec/1.2/spec.html) (for middleware definition)
