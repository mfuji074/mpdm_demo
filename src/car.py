import numpy as np
from enum import Enum, auto

class Policy(Enum):
    KeepLane = auto()
    ChangeLane = auto()

class SubPolicy(Enum):
    KeepVel = auto()
    Accel = auto()
    Decel = auto()

class CarType(Enum):
    Ego = auto()
    Other = auto()

class Car:

    def __init__(self, lane0, pos0, vel0, acc0, vel_nominal, CarType = CarType.Ego, Policy_ini = Policy.KeepLane, SubPolicy_ini = SubPolicy.KeepVel):
        # state
        self.lane = lane0
        self.pos = pos0
        self.vel = vel0
        self.acc = acc0
        self.vel_nominal = vel_nominal # 各レーンでのノミナル速度
        self.vel_lane = 0.0 # レーン方向速度

        self.CarType = CarType

        # policy
        self.Policy = Policy_ini
        self.SubPolicy = SubPolicy_ini
        self.is_lane_changing = False
        self.count_lane_changing = 0

        # measurement
        self.lane_m = [] # 他車とレーンが一致しているか否か(bool)
        self.dst_m = [] # 他車との距離(1D)
        self.rvel_m = [] # 他車との相対速度(1D)
        self.is_car_in_same_lane = 0 # 同じレーンに車がいるか否か
        self.dst_min = []

        # for plotting
        self.lane_his = []
        self.pos_his = []
        self.vel_his = []
        self.acc_his = []
        self.dst_min_his = []


    def measure(self, Car_list):
        # reset measurement
        self.lane_m = []
        self.dst_m = []
        self.rvel_m = []
        self.dst_min = []

        for Car_other in Car_list:
            # レーンが一致しているか否か
            if abs(self.lane - Car_other.lane) < 0.5:
                self.lane_m.append(1)
            else:
                self.lane_m.append(0)

            # 他車との距離
            dst = Car_other.pos - self.pos
            self.dst_m.append(dst)

            # 他車との相対速度
            rvel = Car_other.vel - self.vel
            self.rvel_m.append(rvel)

        # 同じレーンにいる車のインデックス取得
        lane_idx = [i for i, lane in enumerate(self.lane_m) if lane == 1]
        # 同じレーンにいる最も近い車との距離
        try:
            self.dst_min = min([self.dst_m[i] for i in lane_idx if abs(self.dst_m[i])>0.0])
            self.is_car_in_same_lane = 1
        except (ValueError, TypeError):
            self.is_car_in_same_lane = 0


    def exec_policy(self,dt):
        time_for_lane_changing_sec = 10
        car_distance_threshold = 16

        # policy
        if self.Policy == Policy.KeepLane:
            self.vel_lane = 0.0

            if self.CarType == CarType.Other:
                if self.vel < self.vel_nominal[int(self.lane)]:
                    self.SubPolicy = SubPolicy.Accel
                else:
                    self.SubPolicy = SubPolicy.Decel

                if self.is_car_in_same_lane:
                    if abs(self.dst_min) < car_distance_threshold and self.dst_min > 0:
                        self.SubPolicy = SubPolicy.Decel

        elif self.Policy == Policy.ChangeLane:

            if self.count_lane_changing == 0:
                self.is_lane_changing = True
                self.count_lane_changing += 1

                if abs(self.lane) < 1e-6:
                    self.vel_lane = 1/time_for_lane_changing_sec
                else:
                    self.vel_lane = -1/time_for_lane_changing_sec

            elif self.count_lane_changing == int(time_for_lane_changing_sec/dt):
                self.is_lane_changing = False
                self.count_lane_changing = 0
                self.vel_lane = 0.0
                self.Policy = Policy.KeepLane

            else:
                self.count_lane_changing += 1

        # subpolicy
        if self.SubPolicy == SubPolicy.KeepVel:
            self.acc = 0.0

        elif self.SubPolicy == SubPolicy.Accel:
            self.acc = 0.1

        elif self.SubPolicy == SubPolicy.Decel:
            self.acc = -0.1


    def update(self, dt):
        kmh2ms = 1/3.6
        # 状態量更新
        self.pos += self.vel*kmh2ms*dt
        self.vel += self.acc*dt/kmh2ms
        self.lane += self.vel_lane*dt

    def log_state(self):
        self.lane_his.append(self.lane)
        self.pos_his.append(self.pos)
        self.vel_his.append(self.vel)
        self.acc_his.append(self.acc)
        self.dst_min_his.append(self.dst_min)

    def init_log(self):
        self.lane_his = []
        self.pos_his = []
        self.vel_his = []
        self.acc_his = []
        self.dst_min_his = []


