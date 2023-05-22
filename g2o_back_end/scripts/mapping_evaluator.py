import numpy as np
import matplotlib.pyplot as plt
import csv
import math

working_folder = "/code/maps/"
gt_file = "gt_pose.csv"
odom_file = "orig_pose.csv"
opt_file = "opt_pose.csv"

# 读取三个文件
with open(working_folder + gt_file, 'r') as f:
    csv_reader = csv.reader(f, delimiter=' ')
    gt = np.array([[float(aa) for aa in a] for a in csv_reader])

with open(working_folder + odom_file, 'r') as f:
    csv_reader = csv.reader(f, delimiter=' ')
    odom = np.array([[float(aa) for aa in a] for a in csv_reader])

with open(working_folder + opt_file, 'r') as f:
    csv_reader = csv.reader(f, delimiter=' ')
    opt = np.array([[float(aa) for aa in a] for a in csv_reader])

# align timestamp
temp_list = []
for opt_data in opt:
    timestamp = opt_data[0]
    a = gt[:, 0]
    a = a - timestamp
    res_id = np.where(abs(a) < 1e-2)
    if len(res_id[0]) != 0:
        temp_list.append(gt[res_id[0][0]])
    else:
        res_id_less = np.where(a < 0)
        res_id_greater = np.where(a >= 0)
        if len(res_id[0]) != 0:
            # all timestamp greater, use least data directly
            temp_list.append(gt[res_id_greater[0][0]])
        else:
            bef_id = res_id_less[0][-1]
            aft_id = res_id_greater[0][0]
            ts_bef = gt[bef_id][0]
            ts_aft = gt[aft_id][0]
            ratio = (timestamp - ts_bef) / (ts_aft - ts_bef)
            interp_odom = gt[bef_id] + (gt[aft_id] - gt[bef_id]) * ratio
            temp_list.append(interp_odom)
gt_aligned = np.array(temp_list)
print("gt_aligned shape: ", gt_aligned.shape)

temp_list.clear()
for opt_data in opt:
    timestamp = opt_data[0]
    a = odom[:, 0]
    a = a - timestamp
    res_id = np.where(abs(a) < 1e-2)
    if len(res_id[0]) != 0:
        temp_list.append(odom[res_id[0][0]])
    else:
        res_id_less = np.where(a < 0)
        res_id_greater = np.where(a >= 0)
        if len(res_id[0]) != 0:
            # all timestamp greater, use least data directly
            temp_list.append(odom[res_id_greater[0][0]])
        else:
            bef_id = res_id_less[0][-1]
            aft_id = res_id_greater[0][0]
            ts_bef = odom[bef_id][0]
            ts_aft = odom[aft_id][0]
            ratio = (timestamp - ts_bef) / (ts_aft - ts_bef)
            interp_odom = odom[bef_id] + (odom[aft_id] - odom[bef_id]) * ratio
            temp_list.append(interp_odom)
odom_aligned = np.array(temp_list)
print("odom_aligned shape: ", odom_aligned.shape)
if odom_aligned.shape[0] != opt.shape[0] or gt_aligned.shape[0] != opt.shape[0]:
    print("should have same shape!")
    exit(0)


# calculate delta
ori_dis_gt = gt_aligned[0]
ori_dis_opt = opt[0]
ori_dis_odom = odom_aligned[0]

size = opt.shape[0]
odom_error = []
opt_error = []

def dis(line) -> float:
    return math.sqrt(pow(line[1], 2) + pow(line[2], 2) + pow(line[3], 2))

for i in range(size):
    now_dis_gt = gt_aligned[i] - ori_dis_gt
    now_dis_odom = odom_aligned[i] - ori_dis_odom
    now_dis_opt = opt[i] - ori_dis_opt

    odom_error.append(abs(dis(now_dis_odom)-dis(now_dis_gt)))
    opt_error.append(abs(dis(now_dis_opt)-dis(now_dis_gt)))

odom_error = np.array(odom_error)
opt_error = np.array(opt_error)

# calculate statistical data
def cal_mean(data) -> float:
    return np.mean(data)
def cal_rmse(data) -> float:
    return np.sqrt(np.mean(data**2))
odom_error_mean = cal_mean(odom_error)
odom_error_rmse = cal_rmse(odom_error)
opt_error_mean = cal_mean(opt_error)
opt_error_rmse = cal_rmse(opt_error)
print("opt pose error mean: ", opt_error_mean, " rmse: ", opt_error_rmse, " max: ", np.max(opt_error))
print("odom pose error mean: ", odom_error_mean, " rmse: ", odom_error_rmse, " max: ", np.max(odom_error))

y_limit = max(np.max(opt_error), np.max(odom_error)) *1.25

t = opt[:, 0] - float(opt[0, 0])

fig, ax = plt.subplots()
ax.plot(t, opt_error, label='Ours')
ax.plot(t, odom_error, label='Baseline')
ax.set_xlabel('Time(s)')
ax.set_ylabel('ATE(m)')
ax.set_title("Absolute Trajectory Error Comparison")
ax.set_ylim(0, y_limit)
ax.grid(True)
ax.legend()

plt.show()