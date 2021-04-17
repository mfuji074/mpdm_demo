import copy
import numpy as np
import matplotlib.pyplot as plt

from car import Policy, SubPolicy

class MpdmNode:
    def __init__(self, Cars, dt, th, score, policy = [], coef = [10, 100, 1000, 0.8, 0.8, 1.2, 1]):
        self.Cars = Cars
        self.dt = dt
        self.th = th
        self.score = score
        self.policy = policy
        self.coef = coef

        self.child_nodes = None

    def compute_score(self, Cars, previous_policy):
        score = 0.0
        car_ego = Cars[0]

        # 最終位置が大きいほどコスト減
        k1 = self.coef[0]
        if car_ego.MyLoopPass is None:
            score += k1*car_ego.pos_his[0]/(car_ego.pos_his[-1] - car_ego.pos_his[0])
        else:
            dst_traveled = car_ego.pos_his[-1] - car_ego.pos_his[0]
            if dst_traveled < 0:
                dst_traveled += car_ego.MyLoopPass.calc_path_length(1)
            score += k1*car_ego.pos_his[0]/dst_traveled

        # ノミナル速度から離れているとコスト増
        k2 = self.coef[1]
        if car_ego.is_lane_changing:
            score += k2*(car_ego.vel_nominal[0] - car_ego.vel)**2
        else:
            score += k2*(car_ego.vel_nominal[round(car_ego.lane)] - car_ego.vel)**2

        # 障害物（他車）に近いとコスト増
        k3 = self.coef[2]
        safe_distance = 8
        for i,lane_m in enumerate(car_ego.lane_m):
            if lane_m and i > 0:
                for j in range(10):
                    safe_distance_tmp = safe_distance*j/10
                    score += k3*(safe_distance_tmp/(abs(car_ego.dst_m[i])-safe_distance_tmp+1e-8))**2
                score += k3*(1/(abs(car_ego.dst_m[i])+1e-8))**2

        if car_ego.Policy == previous_policy[0] and car_ego.SubPolicy == previous_policy[1]:
            score *= self.coef[3]

        # 走行車線にいるとコスト減
        if abs(car_ego.lane) < 1e-6:
            score *= self.coef[4]

        # 車線変更はコスト増
        if car_ego.Policy == Policy.ChangeLane:
            score *= self.coef[5]

        # 他車の速度が変わるようなポリシーはコスト増
        k4 = self.coef[6]
        for i, car in enumerate(Cars):
            if i > 0:
                score += k4*(car.vel_nominal[round(car.lane)] - car.vel)**2

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
                score += self.compute_score(Cars, previous_policy)

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
                self.child_nodes.append(MpdmNode(Cars_tmp, self.dt, self.th, self.score + score, new_policy, self.coef))

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

    def __init__(self, dt, th, tree_length=1, coef = [10, 100, 1000, 0.8, 0.8]):
        self.dt = dt
        self.th = th # timestep [sec]
        self.tree_length = tree_length
        self.coef = coef

    def optimize(self, Cars):

        # コスト計算用に別オブジェクト生成 + ログの初期化
        Cars_ini = copy.deepcopy(Cars)
        for Car in Cars_ini:
            Car.init_log()

        # 第一ノードを生成し、一段の予測を行う
        score_ini = 0
        root_node = MpdmNode(Cars_ini, self.dt, self.th, score_ini, [], self.coef)
        root_node.expand()

        # 多段ポリシーの予測を行う
        for i in range(self.tree_length-1):
            root_node.expand_end_child_node()

        # 最適なポリシーを取得する
        self.explore_best_policy(root_node, Cars[0])

        return self.best_policy[0]

    def explore_best_policy(self, root_node, ego_car):
        # 末端ノードのスコア、状態、ポリシーを取得
        scores, states, policies = root_node.get_scores_states_policies()

        # スコアから最適ポリシーと状態を取得
        if ego_car.is_lane_changing:
            # 車線変更中はポリシー継続
            for i, policy_seq in enumerate(policies):
                if policy_seq[0][0] == ego_car.Policy and policy_seq[0][1] == ego_car.SubPolicy:
                    index = i
                    break

            self.best_states = states[index]
            self.best_policy = policies[index]

        else:
            best_index = scores.index(min(scores))
            self.best_states = states[best_index]
            self.best_policy = policies[best_index]
