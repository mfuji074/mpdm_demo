import numpy as np
from enum import Enum, auto

class Policy(Enum):
    KeepLane = auto()
    ChangeLane = auto()


class SubPolicy(Enum):
    KeepAcc = auto()
    Accel = auto()
    Decel = auto()


class Car:

    # 車線変更時間遅れ
    step_for_lanechange = 1
    step = 0

    def __init__(self, lane0, pos0, vel0, acc0, vel_nominal = [1.0, 1.2], Policy_ini = Policy.KeepLane, SubPolicy_ini = SubPolicy.KeepAcc):
        # state
        self.lane = lane0
        self.pos = pos0
        self.vel = vel0
        self.acc = acc0
        self.vel_nominal = vel_nominal # 各レーンでのノミナル速度

        # policy
        self.Policy = Policy_ini
        self.SubPolicy = SubPolicy_ini


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

        # 同じレーンにいる車のインデックス取得
        lane_idx = [i for i, lane in enumerate(self.lane_m) if lane == 1]
        # 同じレーンにいる最も近い車との距離
        try:
            self.dst_min = min([abs(self.dst_m[i]) for i in lane_idx if abs(self.dst_m[i])>0.0])
            self.is_car_in_same_lane = 1
        except (ValueError, TypeError):
            self.is_car_in_same_lane = 0


    def exec_policy(self,dt):
        # policy
        if self.Policy == Policy.KeepLane:
            # 特になにもしない
            self.Policy = Policy.KeepLane

        elif self.Policy == Policy.ChangeLane:
            Car.step += 1
            if Car.step == Car.step_for_lanechange:
                if self.lane == 0:
                    self.lane = 1
                    self.Policy = Policy.KeepLane
                else:
                    self.lane = 0
                    self.Policy = Policy.KeepLane

                Car.step = 0

        # subpolicy
        if self.SubPolicy == SubPolicy.KeepAcc:
            self.acc = 0.0

        elif self.SubPolicy == SubPolicy.Accel:
            self.acc = 5e-3

        elif self.SubPolicy == SubPolicy.Decel:
            self.acc = -5e-3


    def update(self, dt):
        # 状態量更新
        self.pos += self.vel*dt
        self.vel += self.acc*dt


    def log_state(self):
        self.lane_his.append(self.lane)
        self.pos_his.append(self.pos)
        self.vel_his.append(self.vel)
        self.acc_his.append(self.acc)
        self.dst_min_his.append(self.dst_min)

    def log_init(self):
        self.lane_his = []
        self.pos_his = []
        self.vel_his = []
        self.acc_his = []
        self.dst_min_his = []


