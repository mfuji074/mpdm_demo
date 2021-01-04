import matplotlib.pyplot as plt
import numpy as np
import copy

from car import Car
from mpdm import MPDM

# simulation period
tf = 5000
dt = 0.1
tspan = np.arange(dt, tf, dt)


# car
car0 = Car(0, 20.0,  0.8, 0.0, [1.0, 1.2]) # lane, pos, vel, acc, vel_max(各レーンでの最高速度)
car1 = Car(0, 100.0, 0.7, 0.0, [0.8, 1.0])
car2 = Car(1, 0.0,  1.0, 0.0, [0.9, 1.1])

cars = [car0, car1, car2]

# MPDM
th = tf/5.0 # horizon
mpdm_car0 = MPDM(dt, th)
interval_mpdm = 3


# simulation
for i in range(len(tspan)):

#    if (i % interval_mpdm) == 0:
#            car[0].policy = mpdm_car0.optimize(cars)

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
    plt.plot(tspan, car.pos_list)

plt.subplot(4,1,2)
for i,car in enumerate(cars):
    plt.plot(tspan, car.vel_list)

plt.subplot(4,1,3)
for i,car in enumerate(cars):
    plt.plot(tspan, car.acc_list)

plt.subplot(4,1,4)
for i,car in enumerate(cars):
    plt.plot(tspan, car.lane_list)

plt.figure()
plt.plot(tspan, cars[1].pos_list - cars[0].pos_list)

plt.show()