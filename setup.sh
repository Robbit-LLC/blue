#!/usr/bin/env bash

export PROJECT_PATH=$(pwd)/..

# Add results of colcon build
source $PROJECT_PATH/blue_ws/install/setup.sh

# Add results of ArduSub build
export PATH=$PROJECT_PATH/ardupilot/build/sitl/bin:$PATH

# Optional: add autotest to the PATH, helpful for running sim_vehicle.py
export PATH=$PROJECT_PATH/ardupilot/Tools/autotest:$PATH

# Add ardupilot_gazebo plugin
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PROJECT_PATH/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH

# Add ardupilot_gazebo models and worlds
export GZ_SIM_RESOURCE_PATH=$PROJECT_PATH/ardupilot_gazebo/models:$PROJECT_PATH/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH

# Add blue models and worlds
export GZ_SIM_RESOURCE_PATH=$PROJECT_PATH/blue_ws/src/blue/blue_description/gazebo/models:$PROJECT_PATH/blue_ws/src/blue/blue_description/gazebo/worlds:$GZ_SIM_RESOURCE_PATH

# Add blue meshes
export GZ_SIM_RESOURCE_PATH=$PROJECT_PATH/blue_ws/src/blue/blue_description/meshes:$GZ_SIM_RESOURCE_PATH

# Build ros_gz for Gazebo Garden
export GZ_VERSION=garden
