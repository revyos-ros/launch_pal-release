# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
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


from launch.launch_context import LaunchContext
from launch.substitution import Substitution


class ConcatSubstitution(Substitution):
    """
    Concatenate list of substitutions and/or strings.

    Example:
    -------
        # Concatenate a Substitution and a string
        config_file_substitution = ConcatSubstitution(
            LaunchConfiguration('robot_name'), '_configuration.yaml'
        )

    """

    def __init__(self, *args: Substitution | str) -> None:
        self.parts = args

    def perform(self, context: LaunchContext) -> str:
        """
        Upon substitution step, perform the concatenation between Substitutions and str.

        :return: str concatenated string
        """
        result: str = ""
        for part in self.parts:
            if isinstance(part, Substitution):
                result += part.perform(context)
            else:
                result += str(part)
        return result
