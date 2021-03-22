import copy
import numpy as np
import matplotlib.pyplot as plt

from car import Policy, SubPolicy

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
        k1 = 10
        score += k1*Car.pos_his[0]/(Car.pos_his[-1] - Car.pos_his[0])

        # ノミナル速度から離れているとスコアダウン
        k2 = 100
        if Car.is_lane_changing:
            #vel_ref = 0.5*(Car.vel_nominal[1] - Car.vel_nominal[0])
            #score += k2*(vel_ref - Car.vel)**2
            score += k2*(Car.vel_nominal[0] - Car.vel)**2
        else:
            score += k2*(Car.vel_nominal[int(Car.lane)] - Car.vel)**2

        # 障害物（他車）から一定距離空けないとスコアダウン
        k3 = 1000
        safe_distance = 10
        if Car.is_car_in_same_lane:
            score += k3*(safe_distance/(abs(Car.dst_min)-safe_distance+1e-8))**2
            score += k3*(1/(Car.dst_min+1e-8))**2

        if Car.Policy == previous_policy[0] and Car.SubPolicy == previous_policy[1]:
            score *= 0.8

        # 走行車線にいるとスコアアップ
        if abs(Car.lane) < 1e-6:
            score *= 0.8

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

    def get_scores_states_policies(self):
        # 末端ノードのスコア、状態、ポリシー列を取得する
        end_child_nodes = []
        scores = []
        states = []
        policies = []

        self.get_end_child_nodes(end_child_nodes)

        for end_child_node in end_child_nodes:
            scores.append(end_child_node.score)
            states.append(end_child_node.Cars)
            policies.append(end_child_node.policy)

        return scores, states, policies


class MPDM:

    def __init__(self, dt, th, tree_length=1):
        self.dt = dt
        self.th = th # timestep [sec]
        self.tree_length = tree_length

    def optimize(self, Cars):

        # コスト計算用に別オブジェクト生成 + ログの初期化
        Cars_ini = copy.deepcopy(Cars)
        for Car in Cars_ini:
            Car.init_log()

        # 第一ノードを生成し、一段の予測を行う
        score_ini = 0
        root_node = MpdmNode(Cars_ini, self.dt, self.th, score_ini)
        root_node.expand()

        # 多段ポリシーの予測を行う
        for i in range(self.tree_length-1):
            root_node.expand_end_child_node()

        # 最適なポリシーを取得する
        self.explore_best_policy(root_node)

        if Cars[0].is_lane_changing:
            # 車線変更中はポリシー継続
            return [Cars[0].Policy, Cars[0].SubPolicy]
        else:
            return self.best_policy[0]

    def explore_best_policy(self, root_node):
        # 末端ノードのスコア、状態、ポリシーを取得
        scores, states, policies = root_node.get_scores_states_policies()

        # スコアから最適ポリシーと状態を取得
        best_index = scores.index(min(scores))
        self.best_states = states[best_index]
        self.best_policy = policies[best_index]
