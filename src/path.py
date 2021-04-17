import matplotlib.pyplot as plt
import numpy as np

class LoopPass:
    def __init__(self, length_lane1, length_straight_line, lane_width = 3.5):
        self.length_lane1 = length_lane1
        self.length_straight_line = length_straight_line
        self.lane_width = lane_width
        self.point_lane1, self.point_lane2, self.r1 = self.design_loop_path(length_lane1, length_straight_line)

    def design_loop_path(self, length_lane1, length_straight_line):

        r1 = (length_lane1 - length_straight_line*2)/2/np.pi
        r2 = r1+self.lane_width

        n = 100
        theta_list = np.linspace(0, 2*np.pi, n)
        point_lane1 = np.zeros((n+1,2))
        point_lane2 = np.zeros((n+1,2))
        for i,theta in enumerate(theta_list):
            if i < len(theta_list)/2:
                origin_x = -r1
                origin_y = length_straight_line
            else:
                origin_x = -r1
                origin_y = 0

            x1 = r1*np.cos(theta) + origin_x
            y1 = r1*np.sin(theta) + origin_y
            x2 = r2*np.cos(theta) + origin_x
            y2 = r2*np.sin(theta) + origin_y

            point_lane1[i,:] = [x1,y1]
            point_lane2[i,:] = [x2,y2]

        point_lane1[-1,:] = point_lane1[0,:]
        point_lane2[-1,:] = point_lane2[0,:]

        return point_lane1, point_lane2, r1

    def plot_loop_path(self):
        plt.figure()
        plt.plot(self.point_lane1[:,0], self.point_lane1[:,1])
        plt.plot(self.point_lane2[:,0], self.point_lane2[:,1])

        plt.show()

    def calc_path_length(self, lane):
        r = self.r1 + self.lane_width*lane
        path_length = self.length_straight_line*2 + 2*np.pi*r
        return path_length

    def conv_pos_lane_in_path(self, pos, lane):
        r = self.r1 + self.lane_width*lane
        length_curve = np.pi * r

        if pos < self.length_straight_line:
            x = self.lane_width*lane
            y = pos
            theta = 0

        elif (self.length_straight_line <= pos) and (pos < (self.length_straight_line + length_curve)):
            theta = (pos - self.length_straight_line)/r
            origin_x = -self.r1
            origin_y = self.length_straight_line

            x = r*np.cos(theta) + origin_x
            y = r*np.sin(theta) + origin_y

        elif ((self.length_straight_line + length_curve) <= pos) and (pos < (self.length_straight_line*2 + length_curve)):
            x = -2*self.r1 - self.lane_width*lane
            y = self.length_straight_line - (pos - self.length_straight_line - length_curve)
            theta = np.pi

        else:
            theta = (pos - self.length_straight_line*2 - length_curve)/r + np.pi
            origin_x = -self.r1
            origin_y = 0

            x = r*np.cos(theta) + origin_x
            y = r*np.sin(theta) + origin_y

        return x, y, theta



if __name__ == '__main__':

    length_lane1 = 200
    length_straight_line = 60
    mypass = LoopPass(length_lane1, length_straight_line)

    mypass.plot_loop_path()