# Copyright (c) 2023 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import collections.abc
import yaml
import ament_index_python as aip
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
from launch.actions import LogInfo
from pathlib import Path

DEFAULT_USER_PARAMETER_PATH = Path(os.environ["HOME"]) / ".pal" / "config"


def get_dotted_value(dotted_arg, d):
    """Access a nested dictionary member using a dotted string."""
    for key in dotted_arg.split('.'):
        if key in d:
            d = d.get(key)
        else:
            return None
    return d


def get_pal_configuration(pkg, node, ld=None, cmdline_args=True):
    """
    Get the configuration for a node from the PAL configuration files.

    :param pkg: The package name
    :param node: The node name
    :param ld: The launch description to log messages to.
                If None, no messages are logged.
    :param cmdline_args: A boolean or a list of arguments that will be added as command-line launch
                arguments. If True (default), all arguments will be added. If False, none will be
                added. If a list, only the arguments in the list will be added.

    :return: A dictionary with the parameters, remappings and arguments
    """
    PAL_USER_PARAMETERS_PATH = Path(os.environ.get(
        'PAL_USER_PARAMETERS_PATH', DEFAULT_USER_PARAMETER_PATH))

    if not PAL_USER_PARAMETERS_PATH.exists():
        if ld:
            ld.add_action(LogInfo(
                msg='WARNING: user configuration path '
                f'{PAL_USER_PARAMETERS_PATH} does not exist. '
                'User overrides will not be available.'))

    # code for recursive dictionary update
    # taken from https://stackoverflow.com/a/3233356
    def update(d, u):
        for k, v in u.items():
            if isinstance(v, collections.abc.Mapping):
                d[k] = update(d.get(k, {}), v)
            else:
                d[k] = v
        return d

    # first, use ament_index to retrieve all configuration files pertaining to a node
    cfg_srcs_pkgs = aip.get_resources(f'pal_configuration.{pkg}')

    cfg_srcs = {}
    for cfg_srcs_pkg, _ in cfg_srcs_pkgs.items():
        cfg_files, _ = aip.get_resource(
            f'pal_configuration.{pkg}', cfg_srcs_pkg)
        for cfg_file in cfg_files.strip().split('\n'):
            share_path = aip.get_package_share_path(cfg_srcs_pkg)
            path = share_path / cfg_file
            if not path.exists():
                if ld:
                    ld.add_action(LogInfo(msg=f'WARNING: configuration file {path} does not exist.'
                                          ' Skipping it.'))
                continue
            if path.name in cfg_srcs:
                if ld:
                    ld.add_action(LogInfo(msg='WARNING: two packages provide the same'
                                          f' configuration {path.name} for {pkg}:'
                                          f' {cfg_srcs[path.name]} and {path}. Skipping {path}'))
                continue
            cfg_srcs[path.name] = path

    config = {}
    for cfg_file in sorted(cfg_srcs.keys()):
        with open(cfg_srcs[cfg_file], 'r') as f:
            config = update(config, yaml.load(f, yaml.Loader))

    if not config:
        return {'parameters': [], 'remappings': [], 'arguments': []}

    # next, look for the user configuration files
    user_cfg_srcs = []
    if PAL_USER_PARAMETERS_PATH.exists():
        # list of (*.yml, *.yaml) in any subdirectory under PAL_USER_PARAMETERS_PATH:
        all_user_cfg_srcs = sorted([f for e in ["*.yml", "*.yaml"]
                                    for f in PAL_USER_PARAMETERS_PATH.glob("**/" + e)])
        for cfg_file in all_user_cfg_srcs:
            with open(cfg_file, 'r') as f:
                content = yaml.load(f, yaml.Loader)
                if not content or not isinstance(content, dict):
                    if ld:
                        ld.add_action(LogInfo(msg=f'WARN: configuration file {cfg_file} is empty'
                                              ' or not a dictionary. Skipping it.'))
                    continue
                for k in content.keys():
                    if node in k:
                        user_cfg_srcs.append(cfg_file)
                        config = update(config, content)

    # finally, return the configuration for the specific node
    node_fqn = None
    for k in config.keys():
        if k.split('/')[-1] == node:
            if not node_fqn:
                node_fqn = k
            else:
                if ld:
                    ld.add_action(LogInfo(msg=f'WARN: found two configuration '
                                          'files with node {node} in different namespaces: '
                                          f'{node_fqn} and {k}.'
                                          f' Ignoring {k} for now, but you probably '
                                          'have an error in your configuration files.'))

    if not node_fqn:
        if ld:
            ld.add_action(LogInfo(msg='ERROR: configuration files found, but'
                                  f' node {node} has no entry!\nI looked into the following'
                                  ' configuration files:'
                                  f' {[str(p) for k, p in cfg_srcs.items()]}\n'
                                  ' Returning empty parameters/remappings/arguments'))
        return {'parameters': [], 'remappings': [], 'arguments': []}

    if cmdline_args:
        if ld is None:
            raise ValueError(
                "cmdline_args can only be used if argument 'ld' is not None")

        # if cmdline_args is True, add all arguments
        if not isinstance(cmdline_args, list):
            cmdline_args = config[node_fqn].setdefault(
                "ros__parameters", {}).keys()

        for arg in cmdline_args:
            default = get_dotted_value(arg, config[node_fqn].setdefault(
                "ros__parameters", {}))
            if default is None:
                ld.add_action(LogInfo(msg=f"WARNING: no default value defined for cmdline "
                                          f"argument '{arg}'. As such, it is mandatory to "
                                          "set this argument when launching the node. Consider "
                                          "adding a default value in the configuration file of "
                                          "the node."))

            ld.add_action(DeclareLaunchArgument(
                arg,
                description=f"Start node and run 'ros2 param describe {node} {arg}' for more "
                            "information.",
                default_value=str(default)))
            config[node_fqn]["ros__parameters"][arg] = LaunchConfiguration(arg, default=[default])

    res = {'parameters': [{k: v} for k, v in config[node_fqn].setdefault('ros__parameters', {})
                          .items()],
           'remappings': config[node_fqn].setdefault('remappings', {}).items(),
           'arguments': config[node_fqn].setdefault('arguments', []),
           }

    if not isinstance(res['arguments'], list):
        res['arguments'] = []
        if ld:
            ld.add_action(LogInfo(msg='ERROR: \'arguments\' field in configuration'
                                  f' for node {node} must be a _list_ of arguments'
                                  ' to be passed to the node. Ignoring it.'))

    if ld:
        ld.add_action(
            LogInfo(msg=f'Loaded configuration for <{node}>:'
                    '\n- System configuration (from higher to lower precedence):\n'
                    + ("\n".join(["\t- " + str(p) for p in sorted(cfg_srcs.values(), reverse=True)]
                                 ) if cfg_srcs else "\t\t- (none)") +
                    '\n- User overrides (from higher to lower precedence):\n'
                    + ("\n".join(["\t- " + str(p) for p in sorted(user_cfg_srcs, reverse=True)]
                                 ) if user_cfg_srcs else "\t- (none)")
                    ))
        if res['parameters']:
            # create an empty launch context to get the default values of the parameters
            lc = LaunchContext()

            param_list = ""
            for d in res['parameters']:
                for k, v in d.items():
                    if isinstance(v, LaunchConfiguration):
                        param_list += f'- {k}: {v.perform(lc)} (can be overridden with {k}:=...)\n'
                    else:
                        param_list += f'- {k}: {v}\n'
            ld.add_action(LogInfo(msg='Parameters:\n' + param_list))
        if res['remappings']:
            ld.add_action(LogInfo(msg='Remappings:\n' +
                                  '\n'.join([f'- {a} -> {b}' for a, b in res['remappings']])))
        if res['arguments']:
            ld.add_action(LogInfo(msg='Arguments:\n' +
                                  '\n'.join([f"- {a}" for a in res['arguments']])))

    return res
