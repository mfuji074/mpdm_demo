import copy

class MPDM:
    def __init__(self, dt, th):
        self.dt = dt # time step
        self.th = th # horizon

    def __compute_score(self, Cars):

        return


    def simulate_forward(self, Cars):
        cost = 0

        for t in range(self.dt, self.th, self.dt):
            # 全車更新
            for car in Cars:
                car.update(self.dt)

            # コスト計算
            cost += __compute_score(Cars[0])

        return cost


    def optimize(self, Cars):
        cost = []

        # コスト計算用に別オブジェクトにする
        Cars_ = copy.deepcopy(Cars)

        # TODO : 抽象化して、for policy in policy_setみたいな感じにする
        Cars_[0].policy = 'keep_lane'
        cost.append(self.simulate_forward(Cars_))

        # コスト計算用に別オブジェクトにする
        Cars_ = copy.deepcopy(Cars)

        Cars_[0].policy = 'change_lane'
        cost.append(self.simulate_forward(Cars_))

        if cost[0] > cost[1]:
            return 'keep_lane'
        else:
            return 'change_lane'
