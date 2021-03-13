#!/usr/bin/env python
#
#
# Author:   Philipp Rothenhaeusler
#           Smart Mobility Lab, KTH
#           Stockholm 2020
#

import os
import sys
import time
import subprocess

# Redirect output to log file since stdoutput is procssed by platformio build system
sys.stdout = open("./log/generic_deploy_key_setup.log", "w")
print('Executing middleware hook...')
if __name__ == "__main__":
    cmds= ("chmod 600 $st_DIR/resources/deploy_keys/*")
    ps = subprocess.Popen(('bash', "../resources/shells/generic_command", cmds), stdout=sys.stdout, stderr=sys.stdout)
    time.sleep(1)
    ps.wait()
    sys.stdout = sys.__stdout__
    sys.exit(0)
