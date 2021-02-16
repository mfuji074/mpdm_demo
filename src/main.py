import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

import numpy as np
import copy

import plotter
from car import Car
from mpdm import MPDM

# simulation period
tf = 800
dt = 0.05
tspan = np.arange(dt, tf, dt)

# car
car0 = Car(0, 0.0,  0.7, 0.0, [1.0, 1.2]) # lane, pos, vel, acc, vel_nominal(各レーンでの最高速度)
car1 = Car(0, 50.0, 0.7, 0.0, [0.8, 1.0])
car2 = Car(1, 20.0,  1.0, 0.0, [0.9, 1.1])
car3 = Car(0, 100.0,  0.8, 0.0, [0.9, 1.15])
car4 = Car(-5000.0, 10000.0,  0.8, 0.0, [0.9, 1.1])

cars = [car0, car1, car2, car3]#, car2, car4]

# MPDM
th = 10 # horizon
mpdm_car0 = MPDM(dt, th, 1, False)
interval_mpdm = 4


# simulation
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
    plt.hlines([0], tspan[0], tspan[-1], "blue", linestyles='dashed')

plt.subplot(5,1,3)
for i,car in enumerate(cars):
    plt.plot(tspan, car.vel_his)

plt.subplot(5,1,4)
for i,car in enumerate(cars):
    plt.plot(tspan, car.acc_his)
plt.subplot(5,1,5)
for i,car in enumerate(cars):
    plt.plot(tspan, car.lane_his)

for i,car in enumerate(cars):
    if i > 0:
        plt.figure()
        pos_tmp = [x - y for (x, y) in zip(cars[i].pos_his, cars[0].pos_his)]
        plt.plot(tspan, pos_tmp)

#plot_cars(cars)

plt.show()

