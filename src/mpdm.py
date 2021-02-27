import copy
import numpy as np
import matplotlib.pyplot as plt

from car import Policy, SubPolicy
from plotter import plot_cars

class MPDM:

    car_dst = 2

    def __init__(self, dt, th, node, isfigure=False):
        self.dt = dt # time step
        self.th = th # horizon
        self.node = node
        self.isfigure = isfigure

    def compute_score(self, Car, policy_set_before):
        score = 0.0

        # 最終位置が大きいほどスコアアップ
        score += 1*Car.pos_his[0]/(Car.pos_his[-1] - Car.pos_his[0])

        # ノミナル速度から離れているとスコアダウン
        score += 10000*(Car.vel_nominal[Car.lane] - Car.vel)**2

        # 障害物（他車）から一定距離空けないとスコアダウン
        if Car.is_car_in_same_lane:
            #score += ((Car.dst_min - MPDM.car_dst)/MPDM.car_dst)**2
            score += 100*(MPDM.car_dst/(Car.dst_min-MPDM.car_dst))**2

        if Car.Policy == policy_set_before[0]:
            score *= 0.88

        if Car.SubPolicy == policy_set_before[1]:
            score *= 0.88

        # 走行車線にいるとスコアアップ
        if Car.lane == 0:
            score *= 0.8

        return score


    def simulate_forward(self, Cars, policy_set_before):
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
            if i > 0:
                score += self.compute_score(Cars[0], policy_set_before)

        return score


    def optimize(self, Cars):
#        score_set = np.zeros(len(Policy)*len(SubPolicy)*self.node)
#        policy_set = np.zeros(len(Policy)*len(SubPolicy), self.node*2)
        score_set = []
        policy_set = []

        if self.isfigure:
            fig = plt.figure()
            ax = fig.add_subplot(1, 1, 1)

        # コスト計算用に別オブジェクトにする
#        Cars_node = copy.deepcopy(Cars)

#        for n in range(self.node):
        policy_set_before = ([Cars[0].Policy, Cars[0].SubPolicy])

        for policy in Policy:
            for subpolicy in SubPolicy:
                # コスト計算用に別オブジェクトにする
                Cars_tmp = copy.deepcopy(Cars)
                for i in range(len(Cars)):
                    Cars_tmp[i].log_init()

                Cars_tmp[0].Policy = policy
                Cars_tmp[0].SubPolicy = subpolicy
                score = self.simulate_forward(Cars_tmp, policy_set_before)
                score_set.append(score)
                policy_set.append([policy, subpolicy])

                if self.isfigure:
                    plot_cars(Cars_tmp, score, ax)
                    fig.savefig('plot.png')

        score_best = min(score_set)
        best_index = score_set.index(score_best)
        return policy_set[best_index]
