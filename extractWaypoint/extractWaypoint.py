#!/usr/bin/env python

import carla
import random
import time 
import csv

red = carla.Color(255, 0, 0)
green = carla.Color(0, 255, 0)
blue = carla.Color(47, 210, 231)
cyan = carla.Color(0, 255, 255)
yellow = carla.Color(255, 255, 0)
orange = carla.Color(255, 162, 0)
white = carla.Color(255, 255, 255)

file_destination = "./data/airport_07_15/"

def road_lane_to_unique_id(road_id, lane_id):
    return road_id * 1000 + lane_id

def main():
    actor_list = []
    
    client = carla.Client('10.214.143.211', 2000)
    client.set_timeout(2.0)

    world = client.get_world()
    map = world.get_map()
    blueprint_library = world.get_blueprint_library()
    debug = world.debug

    topology = map.get_topology()
    lt = 0
    delta_meters = 1
    # 记录已经设置过得unique_id
    unique_id_used = []
    # 转换后的unique_id : [连接的ids]
    pre_ids = dict()
    next_ids = dict()
    # 转换后的unique_id : 路长
    road_length = dict()
    # 转换后的unique_id : [waypoint_ids]
    waypoints_in_lanes = dict()
    for node in topology:
        start = node[0]
        end = node[1]
        # debug.draw_point(start.transform.location + carla.Location(z=0.25), 0.10, blue, lt, False)
        # debug.draw_point(end.transform.location + carla.Location(z=0.25), 0.10, yellow, lt, False)
        unique_id_start = road_lane_to_unique_id(start.road_id, start.lane_id)
        unique_id_end = road_lane_to_unique_id(end.road_id, end.lane_id)
        waypoints_start = start.next_until_lane_end(delta_meters)
        waypoints_start.insert(0, start)
        waypoints_end = end.next_until_lane_end(delta_meters)
        waypoints_end.insert(0, end)
        # 对于start要更新next_ids,road_length,waypoints_in_lanes
        if unique_id_start in unique_id_used:
            # 不是第一次访问到，只更新next_ids即可
            next_ids[unique_id_start].append(unique_id_end)
        else:
            # 第一次访问到该unique_id，都更新
            next_ids[unique_id_start] = [unique_id_end]
            road_length[unique_id_start] = len(waypoints_start) * delta_meters
            waypoints_in_lanes[unique_id_start] = waypoints_start
            unique_id_used.append(unique_id_start)

        # 对于end要更新pre_ids,road_length,waypoints_in_lanes
        if unique_id_end in unique_id_used:
            pre_ids[unique_id_end].append(unique_id_start)
        else:
            pre_ids[unique_id_end] = [unique_id_start]
            road_length[unique_id_end] = len(waypoints_end) * delta_meters
            waypoints_in_lanes[unique_id_end] = waypoints_end
            unique_id_used.append(unique_id_end)    

    # 写入csv文件中
    for unique_id in unique_id_used:
        with open(file_destination + str(unique_id), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(unique_id)
            writer.writerow(road_length[unique_id])
            writer.writerow([0])
            writer.writerow(pre_ids)
            writer.writerow(next_ids)
            for waypoint in waypoints_in_lanes[unique_id]:
                # carla右手系
                data = [waypoint.transform.location.x, -waypoint.transform.location.y, waypoint.transform.location.z, 0, 0, 0, 0]
                writer.writerow(data)


        #  -----------------------------------------------------------------------  #

        # waypoints = []
        # waypoints = start.next_until_lane_end(delta_meters)
        # if waypoints is None:
        #     waypoints = start.previous_until_lane_end(delta_meters)
        #     print("start wp is an end of a road! eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")
        # else:
        #     # 目测都是start点
        #     print("start wp is an start of a road! sssssssssssssssssssssssssssssssssssssssssss")
        # 可视化展示
        # for waypoint in waypoints:
        #     print("waypoint id: %s\troad_id: %s\tsection_id: %s\tlane_id: %s\tis_junction: %s" % (str(waypoint.id), str(waypoint.road_id), str(waypoint.section_id), str(waypoint.lane_id), str(waypoint.is_junction)))    
        #     debug.draw_point(waypoint.transform.location + carla.Location(z=0.25), 0.05, red, lt, False)

        #  -----------------------------------------------------------------------  #

        # waypoints = []
        # waypoints = end.next_until_lane_end(delta_meters)
        # if waypoints is None:
        #     waypoints = start.previous_until_lane_end(delta_meters)
        #     print("end wp is an end of a road! eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")
        # else:
        #     print("end wp is an start of a road! sssssssssssssssssssssssssssssssssssssssssss")
        # 可视化展示
        # for waypoint in waypoints:
        #     print("waypoint id: %s\troad_id: %s\tsection_id: %s\tlane_id: %s\tis_junction: %s" % (str(waypoint.id), str(waypoint.road_id), str(waypoint.section_id), str(waypoint.lane_id), str(waypoint.is_junction)))    
        #     debug.draw_point(waypoint.transform.location + carla.Location(z=0.25), 0.05, green, lt, False)
        # time.sleep(2)

        # raw_input("input a key to continue! -------------------------------------------------")

    print('done.')


if __name__ == '__main__':

    main()
