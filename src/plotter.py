import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def plot_cars(Cars, score, ax):

    for i, car in enumerate(Cars):
        if i == 0:
            color = '#FF0000'
        else:
            color = '#0000FF'

        ax.scatter(car.lane_his[0], car.pos_his[0], s=30, marker="o", c=color)
        ax.set_xlim(-0.05, 1.05)
        if i == 0:
            ax.plot(car.lane_his, car.pos_his, color)
            ax.scatter(car.lane_his[-1], car.pos_his[-1], s=30, marker="^", c=color)


