from gym.envs.registration import register
register(
    id='quad-v0', 
    entry_point='quad.envs:QuadEnv'
)
