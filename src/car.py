class Car:
    def __init__(self, lane0, pos0_in_lane, vel0, acc0, vel_max = [1.0, 1.2], policy = 'keep_lane'):
        # state
        self.lane = lane0
        self.pos_in_lane = pos0_in_lane
        self.vel = vel0
        self.vel_max = vel_max # 各レーンでの最高速度
        self.acc = acc0

        # policy
        self.policy = policy

        # measurement
        self.lane_m = [] # 他車とレーンが一致しているか否か(bool)
        self.dst_m = [] # 他車との距離(1D)
        self.rvel_m = [] # 他車との相対速度(1D)

        # for plotting
        self.lane_list = []
        self.pos_list = []
        self.vel_list = []
        self.acc_list = []


    def measure(self, Car_list):
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
            dst = Car_other.pos_in_lane - self.pos_in_lane
            self.dst_m.append(dst)

            # 他車との相対速度
            rvel = Car_other.vel - self.vel
            self.rvel_m.append(rvel)


    def __exec_policy(self, dt):

        # 同じレーンにいる車のインデックス取得
        lane_idx = [i for i, lane in enumerate(self.lane_m) if lane == 1]
        # 前方にいる車との距離
        try:
            dst = min([self.dst_m[i] for i in lane_idx if self.dst_m[i] > 0.0])
            is_car_in_front = 1
        except (ValueError, TypeError):
            is_car_in_front = 0

        dst_threshold = 3

        # keep lane
        if self.policy == 'keep_lane':
            if is_car_in_front:
                # 車間距離維持
                self.acc += Kp * (e-e1) + Ki * e + Kd * ((e-e1) - (e1-e2))
            else:
                # 最高速度維持
                self.acc += Kp * (e-e1) + Ki * e + Kd * ((e-e1) - (e1-e2))

        # change lane
        elif self.policy == 'change_lane':
            if self.lane == 0:
                self.lane == 1
                self.policy == 'keep_lane'
            else:
                self.lane == 0
                self.policy == 'keep_lane'


    def update(self, dt):
        # ポリシー実行
        self.__exec_policy(dt)

        # 更新
        self.pos_in_lane += self.vel*dt
        self.vel += self.acc*dt


    def log_state(self):
        self.lane_list.append(self.lane)
        self.pos_list.append(self.pos_in_lane)
        self.vel_list.append(self.vel)
        self.acc_list.append(self.acc)


