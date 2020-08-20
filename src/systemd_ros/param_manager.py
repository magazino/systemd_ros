import argparse
import signal
import threading

from systemd.daemon import notify

import rospy
from roslaunch import RLException, ROSLaunchConfig, XmlLoader
from roslaunch.launch import _unify_clear_params


def read_launch(launch_file_name):
    config = ROSLaunchConfig()
    XmlLoader().load(launch_file_name, config, verbose=False)
    return config


class ParamManager(object):

    def __init__(self, launch_file_name):
        self.launch_file_name = launch_file_name
        self.config = read_launch(launch_file_name)
        self.config_lock = threading.Lock()
        self.caller_id = rospy.core.get_caller_id()

        self.load_parameters()

    def multi_call(self, rpc_name, multi_args):
        if not multi_args:
            return
        multi_call_proxy = self.config.master.get_multi()
        rpc = getattr(multi_call_proxy, rpc_name)

        for args in multi_args:
            rpc(self.caller_id, *args)

        for code, msg, _ in multi_call_proxy():
            if code != 1:
                raise RLException("Failed to run {}: {}".format(rpc_name, msg))

    def load_parameters(self):
        with self.config_lock:
            param_server = self.config.master.get()

            self.multi_call('deleteParam', [
                (param, )
                for param in _unify_clear_params(self.config.clear_params)
                if param_server.hasParam(self.caller_id, param)[2]
            ])

            self.multi_call('setParam', [
                (param.key, param.value)
                for param in self.config.params.values()
            ])

    def reload_parameters(self):
        new_config = read_launch(self.launch_file_name)
        with self.config_lock:
            old_config = self.config

            # Delete parameters which are no longer defined
            self.multi_call('deleteParam', [
                (key, )
                for key in old_config.params.keys()
                if key not in new_config.params
            ])

            self.multi_call('setParam', [
                (param.key, param.value)
                for param in new_config.params.values()
            ])
            self.config = new_config


def main():
    rospy.init_node('param_manager', disable_rostime=True)
    parser = argparse.ArgumentParser()
    parser.add_argument('file', type=argparse.FileType('r'))

    args = parser.parse_args(rospy.myargv()[1:])
    args.file.close()

    param_manager = ParamManager(args.file.name)

    signal.signal(signal.SIGHUP,
                  lambda sig, frame: param_manager.reload_parameters())

    notify('READY=1')
    rospy.spin()
