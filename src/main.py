import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

import numpy as np
import time

from car import Car, CarType
from mpdm import MPDM

# simulation period
tf = 300#1200 # sec
dt = 0.5 # sec
tspan = np.arange(dt, tf, dt)

# car
car0 = Car(0, 0.0,  20.0, 0.0, [30.0, 35.0], CarType.Ego) # lane, pos [m], vel [km/h], acc [m/s^2], vel_nominal[km/h] (各レーンでの定常速度)
car1 = Car(0, 50.0, 26.0, 0.0, [26.0, 32.0], CarType.Other)
car2 = Car(1, -120.0,  29.0, 0.0, [29.0, 32.0], CarType.Other)
car3 = Car(0, 100.0,  27.0, 0.0, [28.0, 32.0], CarType.Other)
car4 = Car(1, -70.0,  29.0, 0.0, [29.0, 32.0], CarType.Other)

cars = [car0, car1, car2, car3, car4]

# MPDM
dt_mpdm = dt # timestep, sec
th = 10 # horizon, sec
tree_length = 2
#th = th/tree_length
interval_mpdm = 6 # mpdm execution interval
is_animation = True
is_mp4 = False

cost_coef = [10, 100, 100, 0.8, 0.8, 1.2, 10000] # 距離、速度、車間距離、ポリシー維持バイアス、車線バイアス、車線変更コスト、他車の挙動
mpdm_car0 = MPDM(dt_mpdm, th, tree_length, cost_coef)

if is_animation:
    best_policy_list = []
    best_states_list = []
    fig = plt.figure(figsize=(9,15))
    ax = fig.add_subplot(111)
    ims = []


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

plt.savefig("result.png")

def plot_cars(index):
    # plot best policy for animation
    ax.clear()
    ax.set_xlim(-0.6, 1.6)
    ax.set_ylim(0, best_states_list[-1][0].pos_his[-1]*1.2)
    ax.set_xlabel("Lane", fontsize = 12)
    ax.set_ylabel("Position, m", fontsize = 12)

    for i, car in enumerate(best_states_list[index]):
        if i == 0:
            color = '#FF0000'
        else:
            color = '#0000FF'

        ax.axvline(x=-0.5, color="black", alpha=0.7)
        ax.axvline(x=0.5, color="black", alpha=0.7)
        ax.axvline(x=1.5, color="black", alpha=0.7)

        bias_x = 0.07
        lane_tmp = [lane+i*bias_x for lane in car.lane_his]
        # 車の初期位置
        ax.scatter(lane_tmp[0], car.pos_his[0], s=200, marker="o", c=color)
        # 車の速度
        str_vel = "{:.2f}".format(car.vel_his[0])
        ax.text( lane_tmp[0]+bias_x, car.pos_his[0], f"Velocity = {str_vel} km/h")
        # 自車のポリシー
        if i == 0:
            bias_y = best_states_list[-1][0].pos_his[-1]/50
            for j in range(tree_length):
                ax.text( lane_tmp[0]-0.4, car.pos_his[0]-bias_y, f"{best_policy_list[index][j][0]}")
                ax.text( lane_tmp[0]+0.25, car.pos_his[0]-bias_y, f"{best_policy_list[index][j][1]}")
                bias_y += best_states_list[-1][0].pos_his[-1]/50
        # 車の軌跡
        ax.plot(lane_tmp, car.pos_his, color)
        # 車の終点
        ax.scatter(lane_tmp[-1], car.pos_his[-1], s=100, marker="^", c=color)

if is_animation:
    ani = animation.FuncAnimation(fig, plot_cars, frames=len(best_states_list), interval=1, repeat=True)
    if is_mp4:
        ani.save("mpdm_result.mp4", writer = 'ffmpeg')
    else:
        ani.save("mpdm_result.gif", writer = 'pillow')

plt.show()
