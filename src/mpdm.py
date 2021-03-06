import copy
import numpy as np
import matplotlib.pyplot as plt

from car import Policy, SubPolicy
from plotter import plot_cars

class MpdmNode:
    def __init__(self, Cars, dt, th, score, policy = []):
        self.Cars = Cars
        self.dt = dt
        self.th = th
        self.score = score
        self.policy = policy

        self.child_nodes = None

    def compute_score(self, Car, previous_policy):
        score = 0.0

        # 最終位置が大きいほどスコアアップ
        score += 1*Car.pos_his[0]/(Car.pos_his[-1] - Car.pos_his[0])

        # ノミナル速度から離れているとスコアダウン
        score += 20000*(Car.vel_nominal[Car.lane] - Car.vel)**2

        # 障害物（他車）から一定距離空けないとスコアダウン
        if Car.is_car_in_same_lane:
            #score += ((Car.dst_min - MPDM.car_dst)/MPDM.car_dst)**2
            score += 10000*(MPDM.car_dst/(Car.dst_min-MPDM.car_dst))**2

        if Car.Policy == previous_policy[0]:
            score *= 0.9

        if Car.SubPolicy == previous_policy[1]:
            score *= 0.9

        # 走行車線にいるとスコアアップ
        if Car.lane == 0:
            score *= 0.95

        return score

    def simulate_forward(self, Cars, previous_policy):
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
                score += self.compute_score(Cars[0], previous_policy)

        return score

    def expand(self):
        # ノードを増やす
        self.child_nodes = []

        for policy in Policy:
            for subpolicy in SubPolicy:
                Cars_tmp = copy.deepcopy(self.Cars)
                previous_policy = [Cars_tmp[0].Policy, Cars_tmp[0].SubPolicy]
                Cars_tmp[0].Policy = policy
                Cars_tmp[0].SubPolicy = subpolicy

                score = self.simulate_forward(Cars_tmp, previous_policy)
                new_policy = copy.deepcopy(self.policy)
                new_policy.append([policy, subpolicy])
                self.child_nodes.append(MpdmNode(Cars_tmp, self.dt, self.th, self.score + score, new_policy))

    def expand_end_child_node(self):
        # 末端ノードからノードを増やす
        end_child_nodes = []
        self.get_end_child_nodes(end_child_nodes)

        for end_child_node in end_child_nodes:
            end_child_node.expand()

    def get_end_child_nodes(self, end_child_nodes):
        # 末端ノードを取得する
        for child_node in self.child_nodes:
            if child_node.child_nodes is None:
                end_child_nodes.append(child_node)
            else:
                child_node.get_end_child_nodes(end_child_nodes)

    def get_scores_states_policies(self, scores, states, policies):
        # 末端ノードのスコアおよび状態とポリシー列を取得する
        for child_node in self.child_nodes:
            if child_node.child_nodes is None:
                scores.append(child_node.score)
                states.append(child_node.Cars)
                policies.append(child_node.policy)
            else:
                child_node.get_scores_states_policies(scores, states, policies)


class MPDM:

    car_dst = 2

    def __init__(self, dt, th, tree_length=1):
        self.dt = dt
        self.th = th # timestep [sec]
        self.tree_length = tree_length

        self.policy_num = len(Policy)*len(SubPolicy)
        self.best_states = []
        self.best_policy = []

    def optimize(self, Cars):

        # コスト計算用に別オブジェクト生成
        Cars_ini = copy.deepcopy(Cars)
        for i in range(len(Cars_ini)):
            Cars_ini[i].init_log()

        # 第一ノードを生成し、一段の予測を行う
        score_ini = 0
        root_node = MpdmNode(Cars_ini, self.dt, self.th, score_ini)
        root_node.expand()

        # 多段ポリシーの予測を行う
        for i in range(self.tree_length-1):
            root_node.expand_end_child_node()

        # 最適なポリシーを取得する
        self.explore_best_policy(root_node)

        return self.best_policy[0]

    def explore_best_policy(self, root_node):
        # 最適ノードからポリシー列,状態を取得する
        # 状態からアニメーションを生成する
        scores = []
        states = []
        policies = []

        root_node.get_scores_states_policies(scores, states, policies)

        best_index = scores.index(min(scores))
        self.best_states = states[best_index]
        self.best_policy = policies[best_index]


