from gym.envs.registration import register
register(
    id='quadruped-v0', 
    entry_point='quadruped.envs:QuadrupedEnv'
)
