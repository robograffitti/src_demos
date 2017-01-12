#!/bin/bash
rosbag record /tf -e "/multisense/camera/(left|right)/camera_info" -e "/multisense/camera/(left|right)/image_raw"
# --split --size=7680
