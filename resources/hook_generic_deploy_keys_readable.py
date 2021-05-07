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

if __name__ == "__main__":
    # Redirect output to log file since stdoutput is procssed by platformio build system
    log_dir_path = './log'
    if not os.path.exists(log_dir_path):
        os.mkdir(log_dir_path)
    with open(f"{log_dir_path}/hook_generic_build.log", "w") as sys.stdout:
        sys.stdout = open(log_dir_path + '/generic_deploy_key_setup.log', 'w')
        ##### DEBUG START
        print('Pythonpath: ' + os.path.expandvars('$PYTHONPATH'))
        print("sys.path:\n" + "\n".join(sys.path))
        print("python vesion: " + sys.version)
        ##### DEBUG END
        print('Executing middleware hook...')
        cmds = ("chmod 600 $st_DIR/resources/deploy_keys/*")
    with open(f"{log_dir_path}/hook_generic_build.log", "a") as sys.stdout:
        ps = subprocess.Popen(('bash', "shells/generic_command", cmds),
                              stdout=sys.stdout, stderr=sys.stdout)
        time.sleep(1)
        ps.wait()
        sys.stdout = sys.__stdout__
        sys.exit(0)
