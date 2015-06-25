#!/usr/bin/env python
"""Automates the selection of a suitable classification model by executing a set
of benchmark tests for a set of models.
"""

PKG = "pandora_vision_victim"

import os
import shutil
import readline
import subprocess
import roslib; roslib.load_manifest(PKG)
import rostest
import rospy
import rospkg

# Software License Agreement
__version__ = "0.0.1"
__status__ = "Production"
__license__ = "BSD"
__copyright__ = "Copyright (c) 2015, P.A.N.D.O.R.A. Team. All rights reserved."
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of P.A.N.D.O.R.A. Team nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
__author__ = "Kofinas Miltiadis"
__maintainer__ = "Kofinas Miltiadis"
__email__ = "mkofinas@gmail.com"


def executeBenchmarkTests():
    """Main
    """
    # Set the auto completion scheme
    readline.set_completer_delims(" \t")
    readline.parse_and_bind("tab:complete")

    print "Type the absolute path to the classifier models:"
    print "e.g. /home/user/foo"
    classifiers_path = raw_input("-->")

    package_path = rospkg.RosPack().get_path(PKG)

    package_data_path = os.path.join(package_path, "data")
    package_config_path = os.path.join(package_path, "config")

    for folder in os.listdir(classifiers_path):
        num_of_clusters = folder[7:]
        num_of_clusters = num_of_clusters[:-7]

        training_params_file = os.path.join(package_config_path,
                               "rgb_svm_training_params.yaml")

        new_training_params_file = os.path.join(package_config_path,
                                   "new_rgb_svm_training_params.yaml")

        params_data = open(training_params_file, "r")
        new_params_data = open(new_training_params_file, "w")
        for params_data_line in params_data:
            if "dictionary_size" in params_data_line:
                params_data_line = params_data_line[0:17] + num_of_clusters + "\n"
            new_params_data.write(params_data_line)

        params_data.close()
        new_params_data.close()
        # Remove old params file and replace it with the new one.
        os.remove(training_params_file)
        os.rename(new_training_params_file, training_params_file)

        folder_path = os.path.join(classifiers_path, folder)

        for data_file in os.listdir(folder_path):
            shutil.copyfile(os.path.join(folder_path, data_file),
                            os.path.join(package_data_path, data_file))

        os.chdir("/home/pandora/ws/pandora_ws")

        subprocess.call("rostest pandora_vision_victim victim_benchmark_test.launch",
                        shell=True)
        # Sleep for a while until the process is fully finished.
        rospy.sleep(30)
        print "Finished one test"
        shutil.copyfile(os.path.join(package_data_path, "benchmark_results.txt"),
                        os.path.join(folder_path, "benchmark_results.txt"))
        os.rename(os.path.join(folder_path, "benchmark_results.txt"),
                  os.path.join(folder_path, folder + ".txt"))
    print "Process is finished successfully!"


if __name__ == "__main__":
    executeBenchmarkTests()
