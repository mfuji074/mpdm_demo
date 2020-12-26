class MPDM:
    def __init__(self):
        self.policy = policy
        self.agents = agents
        self.horizon = horizon

    def compute_score(self):



    def simulate_forward(self):
        for t in range(0, self.horizon):
            for agent in self.agents:
                agent.run_policy()


    def optimize(self):
        for policy in policy_set:
            self.simulate_forward()
            self.compute_score()

        return best_policy
