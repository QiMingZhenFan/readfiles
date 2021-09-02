#!/usr/bin/env python
# coding=utf-8
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

file_destination = "./data/airport_07_24/"
# 记录已经设置过得unique_id
# unique_id : order(from 0)
cross_unique_id_used = dict()
lane_unique_id_used = dict()
# 转换后的unique_id : [连接的ids]
pre_ids = dict()
next_ids = dict()
# 转换后的unique_id : 路长
road_length = dict()
# 转换后的unique_id : [waypoint_ids]
waypoints_in_lanes = dict()
# 虚拟的lane和cross，用于面对lane-lane cross-cross情况
virtual_lane = dict()
virtual_cross = dict()

def road_lane_to_unique_id(road_id, lane_id):
    # 不要有0,会产生-1
    return (road_id + 1) * 1000 + lane_id

def unique_ids_to_order(ids, str):
    order = []
    for id in ids:
        if str == 'cross':
            try:
                order.append(cross_unique_id_used[id])
            except:
                if lane_unique_id_used.has_key(id):
                    print("cross id in lane_unique_id_used!")
                else:
                    print("should be a cross, but nothing about this id  %d!" % id)
        elif str == 'lane':
            try:
                order.append(lane_unique_id_used[id])
            except:
                if cross_unique_id_used.has_key(id):
                    print("lane id in cross_unique_id_used!")
                else:
                    print("should be a lane, but nothing about this id  %d!" % id)
    return order

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
    vitual_node_counter = -1

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
        # 应对特殊情况 lane-lane cross-cross
        spectial_case = False
        unique_id_spectial = vitual_node_counter
        if start.is_junction == end.is_junction:
            # 特殊情况，需添加vitual node
            spectial_case = True
            pre_ids[unique_id_spectial] = [unique_id_start]
            next_ids[unique_id_spectial] = [unique_id_end]
            # print("start: %d\tend: %d\tis_junction: %s" %(unique_id_start, unique_id_end, start.is_junction))
            road_length[unique_id_spectial] = delta_meters
            waypoints_in_lanes[unique_id_spectial] = [waypoints_start[-1]]
            waypoints_start.pop()
            if start.is_junction == True:
                # 需要插入lane
                virtual_lane[unique_id_spectial] = len(virtual_lane)
            else:
                # 需要插入cross
                virtual_cross[unique_id_spectial] = len(virtual_cross)
            vitual_node_counter -= 1
            pass
            
        # 对于start要更新next_ids,road_length,waypoints_in_lanes
        if start.is_junction == True:
            unique_id_used = cross_unique_id_used
        else:
            unique_id_used = lane_unique_id_used

        if unique_id_used.has_key(unique_id_start):
            # 不是第一次访问到，只更新next_ids即可
            if spectial_case is not True:
                next_ids[unique_id_start].append(unique_id_end)
            else:
                next_ids[unique_id_start].append(unique_id_spectial)
        else:
            # 第一次访问到该unique_id，都更新
            if spectial_case is not True:
                pre_ids[unique_id_start] = []
                next_ids[unique_id_start] = [unique_id_end]
            else:
                pre_ids[unique_id_start] = []
                next_ids[unique_id_start] = [unique_id_spectial]
            road_length[unique_id_start] = len(waypoints_start) * delta_meters
            waypoints_in_lanes[unique_id_start] = waypoints_start
            unique_id_used[unique_id_start] = len(unique_id_used)

        # 对于end要更新pre_ids,road_length,waypoints_in_lanes
        if end.is_junction == True:
            unique_id_used = cross_unique_id_used
        else:
            unique_id_used = lane_unique_id_used

        if unique_id_used.has_key(unique_id_end):
            if spectial_case is not True:
                pre_ids[unique_id_end].append(unique_id_start)
            else:
                pre_ids[unique_id_end].append(unique_id_spectial)
        else:
            if spectial_case is not True:
                pre_ids[unique_id_end] = [unique_id_start]
                next_ids[unique_id_end] = []
            else:
                pre_ids[unique_id_end] = [unique_id_spectial]
                next_ids[unique_id_end] = []
            road_length[unique_id_end] = len(waypoints_end) * delta_meters
            waypoints_in_lanes[unique_id_end] = waypoints_end
            unique_id_used[unique_id_end] = len(unique_id_used)

    # 统一修改virtual node的order，与正常的衔接上
    counter = len(cross_unique_id_used)
    keys = virtual_cross.keys()
    for key in keys:
        virtual_cross[key] = counter
        counter += 1
    cross_unique_id_used.update(virtual_cross)
    counter = len(lane_unique_id_used)
    keys = virtual_lane.keys()
    for key in keys:
        virtual_lane[key] = counter
        counter += 1
    lane_unique_id_used.update(virtual_lane)

    # 写入csv文件中
    # TODO: 可以直接写成函数，或者起一个公共变量引用
    # for unique_id in cross_unique_id_used.keys():
    #     unique_id_to_order = cross_unique_id_used[unique_id]
    #     # 文件名和id号都要换掉
    #     file_name = 'cross_' + str(unique_id_to_order) + '.csv'
    #     with open(file_destination + file_name, 'w') as f:
    #         writer = csv.writer(f)
    #         writer.writerow([unique_id_to_order])
    #         writer.writerow([road_length[unique_id]])
    #         writer.writerow([0])
    #         writer.writerow(unique_ids_to_order(pre_ids[unique_id], 'lane'))
    #         writer.writerow(unique_ids_to_order(next_ids[unique_id], 'lane'))
    #         for waypoint in waypoints_in_lanes[unique_id]:
    #             # carla右手系
    #             data = [waypoint.transform.location.x, -waypoint.transform.location.y, waypoint.transform.location.z, 0, 0, 0, 0]
    #             writer.writerow(data)
    # for unique_id in lane_unique_id_used.keys():
    #     unique_id_to_order = lane_unique_id_used[unique_id]
    #     # 文件名和id号都要换掉
    #     file_name = 'lane_' + str(unique_id_to_order) + '.csv'
    #     with open(file_destination + file_name, 'w') as f:
    #         writer = csv.writer(f)
    #         writer.writerow([unique_id_to_order])
    #         writer.writerow([road_length[unique_id]])
    #         writer.writerow([0])
    #         writer.writerow(unique_ids_to_order(pre_ids[unique_id], 'cross'))
    #         writer.writerow(unique_ids_to_order(next_ids[unique_id], 'cross'))
    #         for waypoint in waypoints_in_lanes[unique_id]:
    #             # carla右手系
    #             data = [waypoint.transform.location.x, -waypoint.transform.location.y, waypoint.transform.location.z, 0, 0, 0, 0]
    #             writer.writerow(data)


    #  -----------------------------------------------------------------------  #
    if start.is_junction == end.is_junction:
        # check: 可能发生lane连接lane junction连接junction的情况
        # print("junction or not: ", start.is_junction)
        pass
    for node in topology:
        start = node[0]
        end = node[1]

        waypoints = []
        waypoints = start.next_until_lane_end(delta_meters)
        if waypoints is None:
            waypoints = start.previous_until_lane_end(delta_meters)
            print("start wp is an end of a road! eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")
        else:
            # 目测都是start点
            print("start wp is an start of a road! sssssssssssssssssssssssssssssssssssssssssss")
        # 可视化展示
        for waypoint in waypoints:
            # print("waypoint id: %s\troad_id: %s\tsection_id: %s\tlane_id: %s\tis_junction: %s" % (str(waypoint.id), str(waypoint.road_id), str(waypoint.section_id), str(waypoint.lane_id), str(waypoint.is_junction)))    
            # check: 一条lane上都是一种点
            if waypoint.is_junction != start.is_junction:
                # print("waypoint is_junction changed!")
                debug.draw_point(waypoint.transform.location + carla.Location(z=0.25), 0.05, red, lt, False)
            else:
                # time.sleep(0.5)
                debug.draw_point(waypoint.transform.location + carla.Location(z=0.25), 0.05, green, lt, False)

        # #  -----------------------------------------------------------------------  #

        waypoints = []
        waypoints = end.next_until_lane_end(delta_meters)
        if waypoints is None:
            waypoints = start.previous_until_lane_end(delta_meters)
            print("end wp is an end of a road! eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")
        else:
            print("end wp is an start of a road! sssssssssssssssssssssssssssssssssssssssssss")
        # 可视化展示
        for waypoint in waypoints:
            # print("waypoint id: %s\troad_id: %s\tsection_id: %s\tlane_id: %s\tis_junction: %s" % (str(waypoint.id), str(waypoint.road_id), str(waypoint.section_id), str(waypoint.lane_id), str(waypoint.is_junction)))    
            if waypoint.is_junction != end.is_junction:
                # print("waypoint is_junction changed!")
                debug.draw_point(waypoint.transform.location + carla.Location(z=0.25), 0.05, red, lt, False)
            else:
                # time.sleep(0.5)
                debug.draw_point(waypoint.transform.location + carla.Location(z=0.25), 0.05, green, lt, False)
    #     # time.sleep(2)

    # raw_input("input a key to continue! -------------------------------------------------")

    print('done.')


if __name__ == '__main__':

    main()
