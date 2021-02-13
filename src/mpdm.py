import copy
import numpy as np

from car import Policy, SubPolicy

class MPDM:

    car_dst = 1000

    def __init__(self, dt, th, node):
        self.dt = dt # time step
        self.th = th # horizon
        self.node = node

    def compute_score(self, Car):
        score = 0.0

        # 最終位置が大きいほどスコアアップ
        score += Car.pos_his[0]/(Car.pos_his[-1] - Car.pos_his[0])

        # ノミナル速度から離れているとスコアダウン
        score += 100*(Car.vel_nominal[Car.lane] - Car.vel)**2

        # 障害物（他車）から一定距離空けないとスコアダウン
        if Car.is_car_in_same_lane:
            #score += ((Car.dst_min - MPDM.car_dst)/MPDM.car_dst)**2
            score += (MPDM.car_dst/(Car.dst_min-MPDM.car_dst))**2

        # 走行車線にいるとスコアアップ
        if Car.lane == 0:
            score *= 0.9

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

        for policy in Policy:
            for subpolicy in SubPolicy:
                # コスト計算用に別オブジェクトにする
                Cars_tmp = copy.deepcopy(Cars)
                Cars_tmp[0].Policy = policy
                Cars_tmp[0].SubPolicy = subpolicy
                score.append(self.simulate_forward(Cars_tmp))
                policy_set.append([policy, subpolicy])

        score_best = min(score)
        best_index = score.index(score_best)
        return policy_set[best_index]
