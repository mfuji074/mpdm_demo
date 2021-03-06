import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

import numpy as np
import copy
import time

import plotter
from car import Car
from mpdm import MPDM

# simulation period
tf = 600
dt = 0.5
tspan = np.arange(dt, tf, dt)

# car
car0 = Car(0, 0.0,  0.7, 0.0, [1.0, 1.2]) # lane, pos, vel, acc, vel_nominal(各レーンでの最高速度)
car1 = Car(0, 50.0, 0.7, 0.0, [0.8, 1.0])
car2 = Car(1, -10.0,  1.05, 0.0, [1.0, 1.1])
car3 = Car(0, 70.0,  0.8, 0.0, [0.9, 1.15])

cars = [car0, car1, car2, car3]#, car2, car4]

# MPDM
dt_mpdm = dt # timestep [sec]
th = 10 # horizon [sec]
tree_length = 3
mpdm_car0 = MPDM(dt_mpdm, th, tree_length, False)
interval_mpdm = 20


# simulation
start = time.time()
print ("start mpdm...")

for i in range(len(tspan)):

    if (i % interval_mpdm) == 0:
        policy_set = mpdm_car0.optimize(cars)
        cars[0].Policy = policy_set[0]
        cars[0].SubPolicy = policy_set[1]

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
plt.subplot(5,1,1)
for i,car in enumerate(cars):
    plt.plot(tspan, car.pos_his)

plt.subplot(5,1,2)
for i,car in enumerate(cars):
    if i > 0:
        pos_tmp = [x - y for (x, y) in zip(cars[i].pos_his, cars[0].pos_his)]
        plt.plot(tspan, pos_tmp)
    else:
        plt.plot(tspan, np.zeros(len(tspan)), linestyle = "dashed")
    #plt.hlines([0], tspan[0], tspan[-1], "blue", linestyles='dashed')

plt.subplot(5,1,3)
for i,car in enumerate(cars):
    plt.plot(tspan, car.vel_his)

plt.subplot(5,1,4)
for i,car in enumerate(cars):
    plt.plot(tspan, car.acc_his)
plt.subplot(5,1,5)
for i,car in enumerate(cars):
    plt.plot(tspan, car.lane_his)

'''
for i,car in enumerate(cars):
    if i > 0:
        plt.figure()
        pos_tmp = [x - y for (x, y) in zip(cars[i].pos_his, cars[0].pos_his)]
        plt.plot(tspan, pos_tmp)


plt.figure()
plt.plot(cars[0].lane_his, cars[0].pos_his)

for i, car in enumerate(cars):
    if i == 0:
        color = '#FF0000'
    else:
        color = '#0000FF'
    ax.plot(car[i].lane_his, car[i].pos_his, color)
'''

plt.show()
