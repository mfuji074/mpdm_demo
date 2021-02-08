import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

import numpy as np
import copy

import plotter
from car import Car
from mpdm import MPDM

# simulation period
tf = 1e5
dt = 0.1
tspan = np.arange(dt, tf, dt)

# car
car0 = Car(0, 0.0,  0.7, 0.0, [1.0, 1.2], 'keep_lane',) # lane, pos, vel, acc, vel_max(各レーンでの最高速度)
car1 = Car(0, 6000.0, 0.7, 0.0, [0.8, 1.0], 'keep_lane', False)
car2 = Car(1, 0.0,  1.0, 0.0, [0.9, 1.1], 'keep_lane', False)
car3 = Car(1, 300.0,  1.0, 0.0, [0.9, 1.15], 'keep_lane', False)
car4 = Car(0, 10000.0,  0.8, 0.0, [0.9, 1.1], 'keep_lane', False)

cars = [car0, car1, car2, car4]

# MPDM
th = tf/10 # horizon
mpdm_car0 = MPDM(dt, th)
interval_mpdm = 1e4


# simulation
for i in range(len(tspan)):

    if (i % interval_mpdm) == 0:
            cars[0].policy = mpdm_car0.optimize(cars)

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
plt.subplot(4,1,1)
for i,car in enumerate(cars):
    plt.plot(tspan, car.pos_his)

plt.subplot(4,1,2)
for i,car in enumerate(cars):
    plt.plot(tspan, car.vel_his)

plt.subplot(4,1,3)
for i,car in enumerate(cars):
    plt.plot(tspan, car.acc_his)
plt.subplot(4,1,4)
for i,car in enumerate(cars):
    plt.plot(tspan, car.lane_his)

#plot_cars(cars)

plt.show()

