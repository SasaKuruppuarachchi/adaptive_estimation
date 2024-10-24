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

echo "alias sorimo='source /workspaces/adaptive_est/install/setup.bash'" >> ~/.bashrc
echo "alias bilimo='cd /workspaces/adaptive_est && colcon build --packages-skip px4_msgs'" >> ~/.bashrc



echo "alias gitsetlocal="git config --global user.name "SasaKuruppuarachchi" && git config --global user.email "sasa.kuruppuarachchi@gmail.com""" >> ~/.bashrc
echo "alias tree="exa --tree"" >> ~/.bashrc
# Build ROS dependency
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
source /opt/ros/${ROS_DISTRO}/setup.bash

#tentative
#sudo apt-get update
#rosdep update
cd /workspaces/adaptive_est/src/adaptive_estimation/tpc/
pip install .
cd /workspaces/adaptive_est
sudo service udev restart
$@
