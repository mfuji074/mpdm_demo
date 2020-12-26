class Car:
    def __init__(self, lane, pos_in_lane, policy = 'keep_lane'):
        self.lane = lane
        self.pos_in_lane = pos_in_lane

        if lane == 0:
            self.vel = 1
        else:
            self.vel = 1.2

        self.policy = policy

        # for plotting
        self.lane_list = []
        self.pos_list = []
        self.vel_list = []

    def update(self,dt):
        if self.policy == 'keep_lane':
            if self.lane == 0:
                self.vel = 1
            else:
                self.vel = 1.2

        elif self.policy == 'change_lane':
            if self.lane == 0:
                self.lane = 1
                self.vel = 1.2
            else:
                self.lane = 0
                self.vel = 1

        self.pos_in_lane += self.vel*dt

    def log_state(self):
        self.lane_list.append(self.lane)
        self.pos_list.append(self.pos_in_lane)
        self.vel_list.append(self.vel)


