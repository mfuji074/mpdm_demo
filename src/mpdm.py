import copy
import numpy as np

class MPDM:
    def __init__(self, dt, th):
        self.dt = dt # time step
        self.th = th # horizon

    def compute_score(self, Car):
        score = 0.0

        # 最終位置が大きいほどスコアアップ
        score += Car.pos

        # 走行車線にいるとスコアアップ
        if Car.lane == 0:
            score *= 1.1

        return score


    def simulate_forward(self, Cars):
        score = 0
        tspan = np.arange(self.dt, self.th, self.dt)

        for i in range(len(tspan)):
            # 全車更新
            for car in Cars:
                car.measure(Cars)
                car.exec_policy(self.dt)

            for car in Cars:
                car.update(self.dt)
                car.log_state()

        # スコア計算
        score = self.compute_score(Cars[0])

        return score


    def optimize(self, Cars):
        score = []

        # コスト計算用に別オブジェクトにする
        Cars_tmp = copy.deepcopy(Cars)

        # TODO : 抽象化して、for policy in policy_setみたいな感じにする
        Cars_tmp[0].policy = 'keep_lane'
        score.append(self.simulate_forward(Cars_tmp))

        # コスト計算用に別オブジェクトにする
        Cars_tmp = copy.deepcopy(Cars)

        Cars_tmp[0].policy = 'change_lane'
        score.append(self.simulate_forward(Cars_tmp))

        if score[0] > score[1]:
            return 'keep_lane'
        else:
            return 'change_lane'
