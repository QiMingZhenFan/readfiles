# readfiles
read .pcd .bag files 

# 使用指南
- 先 `roslaunch read_bag readBag-tf.launch` 解包bag包中的内容，包含gt的轨迹，以及trajectory对应点的时间
- 再 `roslaunch read_pcd readPcd.laucnh` 解包pcd中的点云数据，只包含x,y,z信息，并与trajectory中的时间点对应，覆盖文件
- 最后 `rosrun read_pcd dataToTUM` 将两个轨迹文件转化为TUM格式的数据（四元数均填0），用于evo的评估