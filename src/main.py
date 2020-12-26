import matplotlib.pyplot as plt
from car import Car

# simulation period
tf = 100
dt = 1
tspan = range(dt, tf, dt)

# car
mycar = Car(0, 0)
car1 = Car(0,5)
car2 = Car(0,10)

car_set = [mycar, car1, car2]


for t in tspan:
    for car in car_set:
        car.update(dt)
        car.log_state()

plt.figure()
for i,car in enumerate(car_set):
    plt.subplot(3,1,i+1)
    plt.plot(car.pos_list)

plt.show()