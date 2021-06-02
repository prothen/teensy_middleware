#!/usr/bin/env python
#
#
# Authors:  Philipp Rothenhaeusler, Stockholm 2020
#           Tobias Bolin, Stockholm 2021
#


Import("env")
import os
import subprocess
import sys
import time
import contextlib
import yaml


def main():
    with work_dir_context() as work_dir:
        log_path = get_log_path(work_dir)
        # Redirect output to log file since stdoutput is procssed by platformio build system
        # with open(log_path, "w") as sys.stdout:
        print('Executing middleware hook...')
        # if len(sys.argv) <= 1:
        #     print('\n\n------------\nERROR:')
        #     print(f'\tOnly received the single input arguments: {sys.argv}')
        #     print('Please state the project while invoking the script.')
        #     sys.exit()
        # if len(sys.argv) <= 2:
        #     print('\n\n------------\nERROR:')
        #     print(f'Received only the arguments: {sys.argv}')
        #     print('Please state the project and middleware implementation.')
        project = 'svea' #sys.argv[1]
        middleware = 'ros1' # sys.argv[2]
        print(f'Received project {project} and middleware {middleware}')
        # Start parsing the configuration
        with open(f"{work_dir}/resources/config.yaml", 'r') as stream:
            try:
                config = yaml.safe_load(stream)
            except yaml.YAMLError as e:
                print('\n\n------------\nERROR:')
                print(f'Encountered exception:\n{e}')
        if config.setdefault(project) is None:
            print('\n\n------------\nERROR:')
            print('Desired project undefined: verify the yaml configuration in middleware.yaml.')
            print(f'Provided middleware.yaml: \n{config}')
            sys.exit(1)
        mw_config = config[project][middleware]
        cmds = (project,)
        print('Parsed configuration: {0}'.format(mw_config))
        if mw_config is None:
            print('No additional ROS message repositories specified.')
        else:
            for rep_config in mw_config.values():
                print(f'Parsing middleware repository entry:\n{rep_config}')
                cmd = ""
                if 'deploy_key' in rep_config:
                    print(f'Deploy key filename: {rep_config["deploy_key"]}')
                    ssh_str = "ssh-agent bash -c 'ssh-add {0}/resources/deploy_keys/{1};"
                    cmd += ssh_str.format(work_dir, rep_config['deploy_key'])
                cmd += "git clone"
                print(f'branch: {rep_config.setdefault("branch")}')
                if 'branch' in rep_config:
                    cmd += f' -b {rep_config["branch"]}'
                    cmd += f' -b {rep_config["branch"]}'
                if 'ref' not in rep_config:
                    print('\n\n------------\nERROR:')
                    print('Missing reference to repository.'
                            'Please specify repository link with the key: ref.')
                    sys.exit(1)
                cmd += f' {rep_config["ref"]}' + "'" if 'deploy_key' in rep_config else ""
                cmds += (cmd,)
        python_path = os.environ["PYTHONPATH"]
        filter_python_path(python_path)
        print('\n------------------------------------------')
        print(f'Parsed repositories: \n{cmds}')
        print('Executing compilation script now')
        print('------------------------------------------\n')

        # with open(log_path, "a") as sys.stdout:
        ps = subprocess.Popen(("bash", f"{work_dir}/resources/shells/ros1_remote_deps",) + cmds,
                              stdout=sys.stdout, stderr=sys.stdout)
        time.sleep(1)
        ps.wait()
        os.environ["PYTHONPATH"] = python_path


@contextlib.contextmanager
def work_dir_context():
    try:
        current_dir = os.getcwd()
        work_dir = decide_work_dir()
        os.chdir(work_dir)
        yield work_dir
    finally:
        os.chdir(current_dir)


def decide_work_dir() -> str:
    """Use the current directory as work_dir if it is """
    def dir_is_valid(dir: str) -> bool:
        return (os.path.exists(f'{current_dir}/platformio.ini')
                and os.path.exists(f'{current_dir}/resources/'))

    current_dir = os.getcwd()
    if dir_is_valid(current_dir):
        work_dir = current_dir
    elif 'tmw_DIR' in os.environ and dir_is_valid('tmw_dir'):
        work_dir = current_dir
    else:
        raise ValueError("No valid working directory found")
    return work_dir


def get_log_path(work_dir: str) -> str:
    log_dir = work_dir + '/log/'
    try:
        file_name = os.path.basename(__file__).replace('.py', '.log')
    except NameError:
        file_name = env['PIOENV'] + '.log'
    log_path = log_dir + file_name
    if not os.path.exists(log_dir):
        os.mkdir(log_dir)
    return log_path


def filter_python_path(python_path: str):
    python_path_list = python_path.split(':')
    if any(['2.7' in p for p in python_path_list]):
        new_python_path_list = [p for p in python_path_list if '3.' not in p]
        new_python_path = ':'.join(new_python_path_list)
        os.environ['PYTHONPATH'] = new_python_path
        print('Temporarily removing platformio PYTHONPATH additions'
              'to ensure python 2.7 compatability')
        print(f'old PYTHONPATH: {python_path}')
        print(f'new PYTHONPATH: {new_python_path}')
        print(f'PYTHONPATH will be restored to old PYTHONPATH'
               ' when ROS message generation is completed')


if __name__ == "SCons.Script":
    main()
