import os
import numpy as np
import time

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

from car import Car, CarType
from mpdm import MPDM

# simulation period
tf = 240#1200 # sec
dt = 0.5 # sec
tspan = np.arange(dt, tf, dt)

# car
car0 = Car(0, 0.0,  20.0, 0.0, [30.0, 35.0], CarType.Ego) # lane, pos [m], vel [km/h], acc [m/s^2], vel_nominal[km/h] (各レーンでの定常速度)
car1 = Car(0, 10.0, 26.0, 0.0, [26.0, 32.0], CarType.Other)
car2 = Car(1, -60.0,  29.0, 0.0, [29.0, 32.0], CarType.Other)
car3 = Car(0, 20.0,  27.0, 0.0, [28.0, 32.0], CarType.Other)
car4 = Car(1, -90.0,  29.0, 0.0, [29.0, 32.0], CarType.Other)

cars = [car0, car1, car2, car3, car4]
#cars = [car0, car1, car3]

# MPDM
dt_mpdm = dt # timestep, sec
th = 10 # horizon, sec
tree_length = 2
#th = th/tree_length
interval_mpdm = 6 # mpdm execution interval
is_animation = True
is_mp4 = False

cost_coef = [1, 1, 1, 1, 0.8, 1, 1] # 距離、速度、車間距離、ポリシー維持バイアス、車線バイアス、車線変更コスト、他車の挙動
mpdm_car0 = MPDM(dt_mpdm, th, tree_length, cost_coef)

if is_animation:
    animation_times = 10 # ×[animation_times] speed, 例えば, animation_times=50のときは実時間の50倍の速度
    best_policy_list = []
    best_states_list = []


# simulation
start = time.time()
print ("start mpdm...")

for i in range(len(tspan)):

    if (i % interval_mpdm) == 0:
        policy_set = mpdm_car0.optimize(cars)
        cars[0].Policy = policy_set[0]
        cars[0].SubPolicy = policy_set[1]
        if is_animation:
            best_policy_list.append(mpdm_car0.best_policy)
            best_states_list.append(mpdm_car0.best_states)

    for car in cars:
        # measurement
        car.measure(cars)

        # policy
        car.exec_policy(dt)

    for car in cars:
        # update state
        car.update(dt)

        # logging
        car.log_state()

elapsed_time = time.time() - start
print ("elapsed_time:{0}".format(elapsed_time) + "[sec]")

# plot
plt.figure()
plt.subplot(4,1,1)
for i,car in enumerate(cars):
    if i > 0:
        pos_tmp = [x - y for (x, y) in zip(cars[i].pos_his, cars[0].pos_his)]
        plt.plot(tspan, pos_tmp)
    else:
        plt.plot(tspan, np.zeros(len(tspan)), linestyle = "dashed")
plt.ylabel("Relative Pos")

plt.subplot(4,1,2)
for i,car in enumerate(cars):
    plt.plot(tspan, car.vel_his)
plt.ylabel("Vel")

plt.subplot(4,1,3)
plt.plot(tspan, cars[0].acc_his)
plt.ylabel("Acc")

plt.subplot(4,1,4)
for i,car in enumerate(cars):
    plt.plot(tspan, car.lane_his)
plt.ylabel("Lane")
plt.xlabel("Time")

dir = './result'
if not os.path.exists(dir):
    os.makedirs(dir)

plt.savefig("./result/result.png")

def plot_cars(index):
    # plot best policy for animation
    ax.clear()
    ax.set_xlim(-0.6, 1.6)
    ax.set_ylim(cars[0].pos_his[index] - 40, cars[0].pos_his[index] + 40)
    ax.set_xlabel("Lane", fontsize = 12)
    ax.set_ylabel("Position, m", fontsize = 12)

    ax.axvline(x=-0.5, color="black", alpha=0.7)
    ax.axvline(x=0.5, color="black", alpha=0.7)
    ax.axvline(x=1.5, color="black", alpha=0.7)

    for i, car in enumerate(cars):
        if i == 0:
            color = '#FF4500'
        else:
            color = '#4169e1'

        # 車の位置
        car_width = 0.5
        car_height = 4.0
        r = patches.Rectangle(xy=(car.lane_his[index]-car_width/2, car.pos_his[index]-car_height/2), width=car_width, height=car_height, ec=color, fc=color)
        ax.add_patch(r)
        # 車の速度
        bias_x = 0.07
        str_vel = "{:.2f}".format(car.vel_his[index])
        ax.text( car.lane_his[index]+bias_x, car.pos_his[index], f"{str_vel} km/h")
        # 自車のポリシー
        if (i == 0) and (index % interval_mpdm == 0):
            bias_y = -2.8
            lane_tmp = best_states_list[int(index/interval_mpdm)][0].lane_his
            pos_tmp = best_states_list[int(index/interval_mpdm)][0].pos_his
            for j in range(tree_length):
                ax.text( lane_tmp[0], pos_tmp[0]+bias_y, f"{best_policy_list[int(index/interval_mpdm)][j][0]}")
                ax.text( lane_tmp[0], pos_tmp[0]+bias_y-0.6, f"{best_policy_list[int(index/interval_mpdm)][j][1]}")
                bias_y += -1.5
            # 車の軌跡
            ax.plot(lane_tmp, pos_tmp, color)
            # 車の終点
            ax.scatter(lane_tmp[-1], pos_tmp[-1], s=100, marker="^", c=color)

def plot_cars_birdeye(index):
    # plot best policy for animation
    ax.clear()
    ax.set_xlim(-0.6, 1.6)
    ax.set_ylim(0, cars[0].pos_his[-1]*1.2)
    ax.set_xlabel("Lane", fontsize = 12)
    ax.set_ylabel("Position, m", fontsize = 12)

    ax.axvline(x=-0.5, color="black", alpha=0.7)
    ax.axvline(x=0.5, color="black", alpha=0.7)
    ax.axvline(x=1.5, color="black", alpha=0.7)

    for i, car in enumerate(cars):
        if i == 0:
            color = '#FF4500'
        else:
            color = '#4169e1'

        bias_x = 0.07
        lane_tmp = [lane+i*bias_x for lane in car.lane_his]
        # 車の初期位置
        ax.scatter(lane_tmp[index], car.pos_his[index], s=200, marker="o", c=color)
        # 車の速度
        str_vel = "{:.2f}".format(car.vel_his[index])
        ax.text( lane_tmp[index]+bias_x, car.pos_his[index], f"Velocity = {str_vel} km/h")
        # 自車のポリシー
        if (i == 0) and (index % interval_mpdm == 0):
            bias_y = best_states_list[-1][0].pos_his[-1]/50
            lane_tmp2 = best_states_list[int(index/interval_mpdm)][0].lane_his
            pos_tmp = best_states_list[int(index/interval_mpdm)][0].pos_his
            for j in range(tree_length):
                ax.text( lane_tmp2[0]-0.4, pos_tmp[0]-bias_y, f"{best_policy_list[int(index/interval_mpdm)][j][0]}")
                ax.text( lane_tmp2[0]+0.25, pos_tmp[0]-bias_y, f"{best_policy_list[int(index/interval_mpdm)][j][1]}")
                bias_y += best_states_list[-1][0].pos_his[-1]/50
            # 車の軌跡
            ax.plot(lane_tmp2, pos_tmp, color)
            # 車の終点
            ax.scatter(lane_tmp2[-1], pos_tmp[-1], s=100, marker="^", c=color)

if is_animation:

    fig = plt.figure(figsize=(4,30))
    ax = fig.add_subplot(111)
    ani = animation.FuncAnimation(fig, plot_cars, frames=len(tspan), interval=dt*1000/animation_times, repeat=True)

    if is_mp4:
        ani.save("./result/mpdm_result.mp4", writer = 'ffmpeg')
    else:
        ani.save("./result/mpdm_result.gif", writer = 'pillow')

    fig = plt.figure(figsize=(4,30))
    ax = fig.add_subplot(111)
    ani = animation.FuncAnimation(fig, plot_cars_birdeye, frames=len(tspan), interval=dt*1000/animation_times, repeat=True)
    if is_mp4:
        ani.save("./result/mpdm_result_birdeye.mp4", writer = 'ffmpeg')
    else:
        ani.save("./result/mpdm_result_birdeye.gif", writer = 'pillow')

plt.show()
