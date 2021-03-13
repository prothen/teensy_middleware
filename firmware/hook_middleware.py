#!/usr/bin/env python
#
#
# Author:   Philipp Rothenhaeusler, Stockholm 2020
#

import sys
import yaml

# Redirect output to log file since stdoutput is procssed by platformio build system
sys.stdout = open("./log/middleware.log", "w")
print('Executing middleware hook...')
if __name__ == "__main__":
    if len(sys.argv) <= 1:
        print('\n\n------------\nERROR:')
        print('\tReceived only the input arguments: {0}'.format(sys.argv))
        print('Please state the project while invoking the script.')
        sys.exit()
    if len(sys.argv) <= 2:
        print('\n\n------------\nERROR:')
        print('Received only the arguments: {0}'.format(sys.argv))
        print('Please state the project and middleware implementation.')
    project = sys.argv[1]
    middleware = sys.argv[2]
    print('Received project {0} and middleware {1}'.format(project, middleware))
    # Start parsing the configuration
    with open("./targets/{0}/config.yaml".format(project),'r') as stream:
        try:
            config = yaml.safe_load(stream)
        except yaml.YAMLError as e:
            print('\n\n------------\nERROR:')
            print('Encountered exception: \n{0}'.format(e))
    if config.setdefault(project) is None:
        print('\n\n------------\nERROR:')
        print('Desired project undefined: verify the yaml configuration in middleware.yaml.')
        print('Provided middleware.yaml: \n{0}'.format(config))
        sys.exit(1)
    mw_config = config[project][middleware]
    cmds = (project,)
    print('Parsed configuration: {0}'.format(mw_config))
    if mw_config is None:
        print('No additional ROS message repositories specified.')
    else:
        for rep_config in mw_config.values():
            print('Parsing middleware repository entry:\n{0}'.format(rep_config))
            cmd = ""
            if rep_config.setdefault('deploy_key'):
                print('Deploy key filename: {0}'.format(rep_config['deploy_key']))
                cmd += "ssh-agent bash -c 'ssh-add $tmw_DIR/resources/deploy_keys/{0};".format(rep_config['deploy_key'])
            cmd += "git clone"
            print('{0}'.format(rep_config.setdefault("branch")))
            if not rep_config.setdefault('branch') is None:
                cmd += " -b {0}".format(rep_config['branch'])
            if rep_config.setdefault('ref') is None:
                print('\n\n------------\nERROR:')
                print('Missing reference to repository. Please specify repository link with the key: ref.')
                sys.exit(1)
            cmd += " {0}".format(rep_config['ref']) + ["'" if rep_config.setdefault('deploy_key') is not None else ""][0]
            cmds += (cmd,)
    print('-----------------------')
    print('Parsed repositories: \n{0}'.format(cmds))
    print('Executing compilation script now')
    import os
    import subprocess
    ps = subprocess.Popen(("bash", "../resources/shells/ros1_deps",) + cmds, stdout=sys.stdout, stderr=sys.stdout)#open(os.devnull, 'w'))
    import time
    time.sleep(1)
    ps.wait()
    #sys.stdout = sys.__stdout__
    sys.exit(0)
