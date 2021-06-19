# import pybullet_envs
import gym
import numpy as np
from sac_torch import Agent
from utils import plot_learning_curve
from gym import wrappers
import quad
from tqdm import tqdm


if __name__ == '__main__':
    env = gym.make('quad-v0')
    agent = Agent(input_dims=env.observation_space.shape, env=env,
            n_actions=env.action_space.shape[0],reward_scale=3)
    n_games = 2000
    # uncomment this line and do a mkdir tmp && mkdir video if you want to
    # record video of the agent playing the game.
    #env = wrappers.Monitor(env, 'tmp/video', video_callable=lambda episode_id: True, force=True)
    filename = 'quad.png'

    figure_file = 'plots/' + filename

    # best_score = env.reward_range[0]
    best_score  = 5000
    score_history = []
    load_checkpoint = True
    learning = False

    if load_checkpoint:
        agent.load_models()
        #env.render(mode='human')

    for i in range(n_games):
        observation = env.reset()
        done = False
        score = 0
        n_step = 0
        while not done:
            action = agent.choose_action(observation)
            observation_new, reward, done, info = env.step(action,n_step)
            score += reward
            n_step += 1
            agent.remember(observation, action, reward, observation_new, done)
            if learning:
                agent.learn()
            observation = observation_new
        score_history.append(score)
        avg_score = np.mean(score_history[-10:])

        if avg_score > best_score:
            best_score = avg_score
            if learning:
                agent.save_models()

        print('episode ', i, 'score %.1f' % score, 'avg_score %.1f' % avg_score)
        if learning and i%2==0 and i>0:
            x = [j+1 for j in range(i)]
            plot_learning_curve(x, score_history, figure_file)
    if learning:
        x = [i+1 for i in range(n_games)]
        plot_learning_curve(x, score_history, figure_file)
    
