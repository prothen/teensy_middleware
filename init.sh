#!/bin/bash
#
# Convenience setup tool for platformio build tool.
#
# This script can be invoked location indepdently.
#
# Author: Philipp Rothenhäusler, KTH Stockholm 2020
#

cw="$(readlink -m "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd )")"
source $cw/resources/shells/teensy_mw_watermark
echo "SETUP: Start configuration of tool chain:"


echo "UDEV: Test if rules installed:"
if ! [ -f "/etc/udev/rules.d/99-teensy-mw.rules" ]; then
    echo -e "\tNo Teensy dynamic device management rules installed!\n"
    echo -e "\tCopying now... (haz sudo plz)"
    sudo cp $cw/resources/99-teensy-mw.rules /etc/udev/rules.d/
    if ! [ -f "/etc/udev/rules.d/99-teensy-mw.rules" ]; then
        echo -e "\tFailed to copy. Maybe sudo usage denied? \nManual intervention required: Copy 99-teensy-mw.rules to /etc/udev/rules.d/. Then invoke ./init.sh once more."
        echo "UDEV: FAILED."
        exit
    fi
    echo "UDEV: Installed udev rules successfully. Physically reconnect the device now in order to allow automatic boot mode selection."
    echo -e "\t-> Dynamic device management setup completed."
    echo "UDEV: DONE."
else
    echo -e "UDEV: Found udev rules.\n -> Dynamic device management already completed."
    echo "UDEV: DONE."
fi

[[ -d ~/.platformio/penv ]] && echo "PlatformIO Virtual environment detected. Activating" && source ~/.platformio/penv/bin/activate

# Basic platformio setup
pip3 install -qr $cw/requirements.txt

# Possibly more dependencies resolved subsequently

echo "SETUP: COMPLETED."
