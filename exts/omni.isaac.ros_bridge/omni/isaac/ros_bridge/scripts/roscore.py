# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import signal
import subprocess
import sys

import carb
import omni
import psutil


def kill_child_processes(parent_pid, sig=signal.SIGKILL):
    omni.kit.app.get_app().print_and_log("Try to kill child pids of roscore pid: " + str(parent_pid))
    try:
        parent = psutil.Process(parent_pid)
    except psutil.NoSuchProcess:
        return
    children = parent.children(recursive=True)
    for process in children:
        omni.kit.app.get_app().print_and_log(f"Stopping Roscore: killing child: {process}")
        process.send_signal(sig)
    parent.kill()


class Roscore(object):
    """
    roscore wrapped into a subprocess.
    Singleton implementation prevents from creating more than one instance.
    """

    __initialized = False

    def __init__(self):
        self.roscore_process = None
        self.roscore_pid = None
        if Roscore.__initialized:
            carb.log_warn("Roscore object was already created, skipping")
            return
        Roscore.__initialized = True

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.ros_bridge")
        self._ros_extension_path = ext_manager.get_extension_path(ext_id)
        kit_folder = carb.tokens.get_tokens_interface().resolve("${kit}")
        self._startup(kit_folder + "/python/bin", self._ros_extension_path + "/noetic", "_CATKIN_SETUP_DIR")

    def __del__(self):
        self.shutdown()

    def _check_running(self):
        import rosgraph

        try:
            rosgraph.Master("/rostopic").getPid()
        except:
            omni.kit.app.get_app().print_and_log("ROS master is not running")
            return False
        else:
            omni.kit.app.get_app().print_and_log("ROS master is already running")
            return True

    def _startup(self, python_path, ros_path, prefix):

        ros_env = {}
        if self._check_running() == True:
            omni.kit.app.get_app().print_and_log("Roscore already running, not starting internal roscore")
            return
        omni.kit.app.get_app().print_and_log("Starting Roscore...")
        # Get ros env parameters from bash setup script
        try:
            self.bash_process = subprocess.check_output(
                f"export {prefix}={ros_path}; source {ros_path}/setup.sh; env -0", shell=True, executable="/bin/bash"
            )
            for line in self.bash_process.decode(sys.stdout.encoding).split("\0"):
                result = line.split("=")
                if len(result) == 2:
                    if result[0] in ["CMAKE_PREFIX_PATH", "LD_LIBRARY_PATH", "PATH", "PKG_CONFIG_PATH", "PYTHONPATH"]:
                        ros_env[result[0]] = result[1]

        except OSError as e:
            carb.log_error("roscore could not be run")
            self.shutdown()
            raise e

        try:
            # append path to specified python bin folder
            ros_env["PATH"] = f"{python_path}:{ros_path}/bin:/bin"
            # append path to where python libs are stored
            ros_env["PYTHONPATH"] += f":{ros_path}/local/lib/python3.10/dist-packages"
            # print(ros_env)
            # running roscore will output logs, rosmaster --core disables rosout
            self.roscore_process = subprocess.Popen(
                ["exec " + "rosmaster --core"], shell=True, cwd=f"{ros_path}", env=ros_env
            )
            self.roscore_pid = self.roscore_process.pid  # pid of the roscore process (which has child processes)
        except OSError as e:
            carb.log_error("roscore could not be run")
            self.shutdown()
            raise e

    def shutdown(self):
        if Roscore.__initialized is False:
            # Shutdown was already called
            omni.kit.app.get_app().print_and_log("shutdown already called")
            return
        if self.roscore_pid is not None and self.roscore_process is not None:
            # kill_child_processes(self.roscore_pid)
            self.roscore_process.kill()
            # omni.kit.app.get_app().print_and_log("waiting for process to finish: " + str(self.roscore_pid))
            # self.roscore_process.wait()  # important to prevent from zombie process
            self.roscore_process = None
            self.roscore_pid = None
            omni.kit.app.get_app().print_and_log("Roscore shutdown complete")
        else:
            carb.log_warn("Roscore was not started by this object or already shutdown, couldn't stop")
        Roscore.__initialized = False
