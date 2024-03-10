import numpy as np
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('TkAgg') 

# 绘制时间-车速图
with open('/home/sun234/pid_lon_mpc_lat/src/pid_lon_mpc_lat/data/U_road.txt', mode='r') as f1:
    list_1 = f1.readlines()
list_2 = [float(list_i) for list_i in list_1]
fig1 = plt.figure(dpi = 80)
plt.plot(np.arange(0.0, len(list_2) * 0.02, 0.02), list_2)
plt.xlabel("time/s")
plt.ylabel("v/[m/s]")
plt.savefig('/home/sun234/pid_lon_mpc_lat/src/pid_lon_mpc_lat/figure_result/U_road.png')
plt.show()

# 提取车速偏差，横摆角偏差，横向偏差，程序运行时间
with open('/home/sun234/pid_lon_mpc_lat/src/pid_lon_mpc_lat/data/U_road_error.txt', mode='r') as f1:
    list_error = f1.readlines()
list_error_v, list_error_eyaw, list_error_ey, list_cal_time = [], [], [], []
for line in list_error:
        list_error_v.append(float(line.split(',')[0]))
        list_error_eyaw.append(float(line.split(',')[1]))
        list_error_ey.append(float(line.split(',')[2]))
        list_cal_time.append(float(line.split(',')[3]))

# 绘制时间-车速偏差图
fig2 = plt.figure(dpi = 80)
plt.plot(np.arange(0.0, len(list_error_v) * 0.02, 0.02), list_error_v)
plt.xlabel("time/s")
plt.ylabel("error v/[m/s]")
plt.savefig('/home/sun234/pid_lon_mpc_lat/src/pid_lon_mpc_lat/figure_result/U_road_error_v.png')
plt.show()

fig3 = plt.figure(dpi = 80)
plt.plot(np.arange(0.0, len(list_error_eyaw) * 0.02, 0.02), list_error_eyaw)
plt.xlabel("time/s")
plt.ylabel("error yaw/rad")
plt.savefig('/home/sun234/pid_lon_mpc_lat/src/pid_lon_mpc_lat/figure_result/U_road_error_yaw.png')
plt.show()

fig4 = plt.figure(dpi = 80)
plt.plot(np.arange(0.0, len(list_error_ey) * 0.02, 0.02), list_error_ey)
plt.xlabel("time/s")
plt.ylabel("error y/m")
plt.savefig('/home/sun234/pid_lon_mpc_lat/src/pid_lon_mpc_lat/figure_result/U_road_error_y.png')
plt.show()

fig5 = plt.figure(dpi = 80)
plt.plot(np.arange(0.0, len(list_cal_time) * 0.02, 0.02), list_cal_time)
plt.xlabel("time/s")
plt.ylabel("program runtime/s")
plt.savefig('/home/sun234/pid_lon_mpc_lat/src/pid_lon_mpc_lat/figure_result/U_road_program_runtime.png')
plt.show()

