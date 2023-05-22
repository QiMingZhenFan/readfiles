#!/bin/bash

set -e

# # 获取当前路径
# current_path=$PWD
# echo "当前路径: $current_path"

# # 切换到上一级目录
# cd ..
# parent_path=$PWD
# echo "上一级目录: $parent_path"

catkin_make -DCATKIN_WHITELIST_PACKAGES="g2o_back_end" -DCMAKE_EXPORT_COMPILE_COMMANDS=1
source devel/setup.bash

# 获取脚本文件相对于终端的路径
script_path=$(dirname "$0")
echo "脚本文件所在的路径: $script_path"

cd $script_path
current_path=$PWD
echo "当前路径: $current_path"

roslaunch ../launch/run_mapping.launch
python3 ./mapping_evaluator.py