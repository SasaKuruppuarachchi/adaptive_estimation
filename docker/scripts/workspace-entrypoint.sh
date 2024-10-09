#!/bin/bash
#
# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

# tmux
git clone https://github.com/tmux-plugins/tpm ~/.tmux/plugins/tpm
wget https://raw.githubusercontent.com/SasaKuruppuarachchi/SasaKuruppuarachchi/main/.tmux.conf -P ~/
echo "alias sorpx4='source /workspaces/aerostack2_ws/install/setup.bash'" >> ~/.bashrc

echo "alias runimo='cd /workspaces/learned_io/src/learned_inertial_model_odometry/ && python learned_inertial_model_odometry/main_filter.py --base_dir=/workspaces/learned_io/src/learned_inertial_model_odometry --root_dir=datasets --out_dir=results --dataset=Blackbird --data_list=test.txt --checkpoint_fn=net_blackbird.pt --model_param_fn=model_net_parameters_net_blackbird.json'" >> ~/.bashrc
echo "alias plotimo='cd /workspaces/learned_io/src/learned_inertial_model_odometry/ && python learned_inertial_model_odometry/filter/python/plot_filter_output.py --base_dir=/workspaces/learned_io/src/learned_inertial_model_odometry --dataset_dir=datasets --result_dir=results --dataset=Blackbird --seq=clover/yawForward/maxSpeed5p0/test'" >> ~/.bashrc
echo "alias rosimo='ros2 run learned_inertial_model_odometry imo_filter_node --base_dir=/workspaces/learned_io/src/learned_inertial_model_odometry --root_dir=datasets --out_dir=results --dataset=Blackbird --data_list=test.txt --checkpoint_fn=net_blackbird.pt --model_param_fn=model_net_parameters_net_blackbird.json'" >> ~/.bashrc

echo "alias sorimo='source /workspaces/learned_io/install/setup.bash'" >> ~/.bashrc
echo "alias bilimo='cd /workspaces/learned_io && colcon build --packages-skip px4_msgs'" >> ~/.bashrc

echo "alias sorcon='source /workspaces/agipix_control/install/setup.bash'" >> ~/.bashrc

# Build ROS dependency
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
source /opt/ros/${ROS_DISTRO}/setup.bash

#tentative
#sudo apt-get update
#rosdep update
cd /workspaces/learned_io
sudo service udev restart
mamba init
$@
