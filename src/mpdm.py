import copy
import numpy as np

from car import Policy, SubPolicy

class MPDM:
    def __init__(self, dt, th):
        self.dt = dt # time step
        self.th = th # horizon
        #self.th_s = th_s # subhorizon

    def compute_score(self, Car):
        score = 0.0

        # 最終位置が大きいほどスコアアップ
        score += Car.pos

        # ノミナル速度から離れているとスコアダウン
        score -= (Car.vel_nominal[Car.lane] - Car.vel)**2

        # 障害物（他車）から遠いとスコアアップ
        score += Car.dst_min**2

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
        policy_set = []

        # コスト計算用に別オブジェクトにする
        Cars_tmp = copy.deepcopy(Cars)

        for policy in Policy:
            for subpolicy in SubPolicy:
                Cars_tmp[0].Policy = policy
                Cars_tmp[0].SubPolicy = subpolicy
                score.append(self.simulate_forward(Cars_tmp))
                policy_set.append([policy, subpolicy])

        score_max = max(score)
        max_index = score.index(score_max)
        return policy_set[max_index]
