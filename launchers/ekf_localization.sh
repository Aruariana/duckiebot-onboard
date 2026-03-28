#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launching app
dt-exec roslaunch --wait duckietown_demos ekf_localization.launch

# wait for app to end
dt-launchfile-join
