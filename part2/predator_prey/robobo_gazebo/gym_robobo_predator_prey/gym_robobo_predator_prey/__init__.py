from gym.envs.registration import register

register(
    id='gym_robobo_predator_prey-v0',
    entry_point='gym_robobo_predator_prey.envs:RoboboPredatorPreyEnv',
)

register(
    id='gym_robobo_predator_prey_real-v0',
    entry_point='gym_robobo_predator_prey.envs:RoboboPredatorPreyEnvReal',
)
