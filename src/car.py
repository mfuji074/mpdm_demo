import numpy as np

class Car:

    # 車間距離
    dst_threshold = 1000

    def __init__(self, lane0, pos0, vel0, acc0, vel_max = [1.0, 1.2], policy = 'keep_lane', is_closing = True):
        # state
        self.lane = lane0
        self.pos = pos0
        self.vel = vel0
        self.acc = acc0
        self.vel_max = vel_max # 各レーンでの最高速度

        # policy
        self.policy = policy
        self.is_closing = is_closing # 前方に車がいるとき、距離を詰めるか否か

        # for pid control
        self.ei_kv = 0.0
        self.ed_kv = 0.0
        self.ei_kd = 0.0
        self.ed_kd = 0.0

        # measurement
        self.lane_m = [] # 他車とレーンが一致しているか否か(bool)
        self.dst_m = [] # 他車との距離(1D)
        self.rvel_m = [] # 他車との相対速度(1D)

        # for plotting
        #self.lane_his = np.empty(0)
        #self.pos_his = np.empty(0)
        #self.vel_his = np.empty(0)
        #self.acc_his = np.empty(0)
        self.lane_his = []
        self.pos_his = []
        self.vel_his = []
        self.acc_his = []


    def measure(self, Car_list):
        # reset measurement
        self.lane_m = []
        self.dst_m = []
        self.rvel_m = []

        for Car_other in Car_list:
            # レーンが一致しているか否か
            if self.lane == Car_other.lane:
                self.lane_m.append(1)
            else:
                self.lane_m.append(0)

            # 他車との距離
            dst = Car_other.pos - self.pos
            self.dst_m.append(dst)

            # 他車との相対速度
            rvel = Car_other.vel - self.vel
            self.rvel_m.append(rvel)


    def exec_policy(self,dt):

        # 同じレーンにいる車のインデックス取得
        lane_idx = [i for i, lane in enumerate(self.lane_m) if lane == 1]
        # 前方にいる車との距離
        try:
            dst = min([self.dst_m[i] for i in lane_idx if self.dst_m[i] > 0.0])
            is_car_in_front = 1
        except (ValueError, TypeError):
            is_car_in_front = 0


        # keep lane
        if self.policy == 'keep_lane':
            if is_car_in_front and dst < Car.dst_threshold*1.2  and self.is_closing:
                # 車間距離維持
                # 最高速度維持
                Kp = 1e-7
                Kd = 1e-3
                e =  dst - Car.dst_threshold
                self.acc = Kp * e + Kd * (e - self.ed_kd)/dt
                self.ed_kd = e
            else:
                # 最高速度維持
                Kp = 1e-2
                Kd = 1e-6
                e =  self.vel_max[self.lane] - self.vel
                self.acc = Kp * e + Kd * (e - self.ed_kv)/dt
                self.ed_kv = e

        # change lane
        elif self.policy == 'change_lane':
            if self.lane == 0:
                self.lane = 1
                self.policy = 'keep_lane'
            else:
                self.lane = 0
                self.policy = 'keep_lane'


    def update(self, dt):
        # 状態量更新
        self.pos += self.vel*dt
        self.vel += self.acc*dt


    def log_state(self):
        #self.lane_his = np.append(self.lane_his, self.lane)
        #self.pos_his = np.append(self.pos_his, self.pos)
        #self.vel_his = np.append(self.vel_his, self.vel)
        #self.acc_his = np.append(self.acc_his, self.acc)
        self.lane_his.append(self.lane)
        self.pos_his.append(self.pos)
        self.vel_his.append(self.vel)
        self.acc_his.append(self.acc)


