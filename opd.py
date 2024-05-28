import sys
import os
import time
import gym
from sympy import Nor


import sofagym
from sofagym.envs import *

import psutil
pid = os.getpid()
py = psutil.Process(pid)

sys.path.insert(0, os.getcwd()+"/..")

__import__('sofagym')


import datetime
from pathlib import Path
#import gymnasium as gym
import json
from docopt import docopt
from itertools import product
from multiprocessing.pool import Pool

from rl_agents.trainer import logger
from rl_agents.trainer.evaluation import Evaluation
from rl_agents.agents.common.factory import load_agent, load_environment
from rl_agents.agents.tree_search.deterministic import DeterministicPlannerAgent


from stable_baselines3.common.vec_env import (SubprocVecEnv,
                                              VecMonitor, VecVideoRecorder,
                                              sync_envs_normalization)
from stable_baselines3.common.vec_env.vec_normalize import VecNormalize
from agents.utils import make_env, mkdirp, sec_to_hours


env_name = 'catheter_beam_1_instrument-v0'
env_seed = 0

if __name__ == '__main__':
    env = gym.make(env_name)
    #env = NormalizeReward(env)
    env.seed(env_seed)
    env.reset()
    env.save_config()
    env.save_step(0)

    #vec_env = SubprocVecEnv([make_env(env_name, 0, env_seed, config={"render": 1})])

    # agent_env = gym.make(env_name)
    # #env = NormalizeReward(env)
    # agent_env.seed(env_seed)
    # agent_env.reset()
    # agent_env.load_config(env.config_file)

    agent = DeterministicPlannerAgent(env)

    #env.render()

    evaluation = Evaluation(env,
                            agent,
                            run_directory="/home/samuelyoussef/Documents/SOFA/Plugins/SofaGym/sofagym/envs/CatheterBeam1Instrument/Results/save/nodes",
                            num_episodes=1,
                            sim_seed=0,
                            display_env=True,
                            display_agent=False,
                            display_rewards=False
                            )

    evaluation.train()
    evaluation.test()

    # actions = agent.plan(obs)
