import math
import pathlib
import sys

import numpy as np
import Sofa
import Sofa.Core
import Sofa.Simulation
import SofaRuntime
from splib3.animation.animate import Animation

from agents.SB3Agent import SB3Agent
from stable_baselines3 import A2C, DDPG, DQN, PPO, SAC, TD3
import sofagym
from sofagym.envs import *
from colorama import Fore
import pickle
import os
import gym

from stable_baselines3.common.vec_env import (SubprocVecEnv,
                                              VecMonitor, VecVideoRecorder,
                                              sync_envs_normalization)
from stable_baselines3.common.vec_env.vec_normalize import VecNormalize
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.evaluation import evaluate_policy

from agents.utils import make_env, mkdirp, sec_to_hours


from sofagym.envs.CatheterBeam1Instrument.CatheterBeam1InstrumentToolbox import getState

sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute())+"/../")
sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute()))

SofaRuntime.importPlugin("Sofa.Component")

class LoadModel(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        
        self.rootNode = None
        if kwargs["rootNode"]:
            self.rootNode = kwargs["rootNode"]

        self.model_dir = None
        if kwargs["model_dir"]:
            self.model_dir = kwargs["model_dir"]

        self.first_step = True
        self.eval_env = None
        self.eval_model = None
        self.observation = None

    '''
    def onKeypressedEvent(self, event):
        key = event['key']

        if key=="E":
            Sofa.Simulation.animate(self.rootNode, 0.01)
            print(f"-------------------BEGIN {self.first_step}")
            self.load()
            self.first_step += 1
            
        return 0
    '''
    
    '''
    def onEvent(self, event):
        #print(event)

        if event['type'] == "SimulationInitDoneEvent":
            self.load()
            self.observation = self.eval_env.reset()
            print("-------------------BEGIN SIMULATION")
    '''        

    '''
    def onSimulationInitStartEvent(self):
        #self.load()
        #self.observation = self.eval_env.reset()
        print("-------------------BEGIN SIMULATION")
    '''
        
    def onAnimateBeginEvent(self, dt):
        print("-------------------BEGIN ANIMATION")
        if self.first_step:
            print(f"-------------------STEP IN {self.first_step}")
            self.load()
            self.observation = self.eval_env.reset()
            print("------------------------------------RESET", self.observation)
            self.first_step = False
        
        print(f"-------------------STEP OUT {self.first_step}")
        
        print("-------------------BEGIN ANIMATE")
        reward, done = self.evaluate()

        #return self.observation, reward, done
    
    #def onAnimateEndEvent(self, dt):
    #    self.first_step += 1
    #    print(f"-------------------STEP OUT 2 {self.first_step}")
    
    def evaluate(self):
        #agent = SB3Agent.load(self.mod
        #agent.eval(5, model_timestep='best_model', render=False, record=False)el_dir)
        #_, _ = evaluate_policy(eval_model, eval_env, n_eval_episodes=1,
        #                       deterministic=True, render=False)

        action, state = self.eval_model.predict(self.observation, deterministic=True)
        print("-------------------------PREDICT", action)
        self.observation, reward, done, info = self.eval_env.step(action)
        print("--------------------------STEP", self.observation, done)

        if done:
            #root = Sofa.Simulation.load('/home/samuelyoussef/Documents/SOFA/Plugins/SofaGym/sofagym/no_server/envs/CartPole/CartPoleScene.py')
            #self.rootNode = root
            #Sofa.Simulation.init(self.rootNode)
            "--------------------------------------DONE"
            self.observation = self.eval_env.reset()

        return reward, done
    
    def load(self, model_timestep='best_model'):
        """Load a pre-trained model.
        
        Parameters
        ----------
        model_dir: str
            The directory where the model is saved.
        model_timestep: str, default='latest_model'
            The checkpoint of the model to load.
        
        Returns
        -------
        agent : object
            The loaded Stable Baselines3 agent with the pre-trained model.
        """
        checkpoint_dir = f"{self.model_dir}/model"
        checkpoint_path = f"{checkpoint_dir}/{model_timestep}"
        if not os.path.exists(checkpoint_path + ".zip"):
            print(Fore.RED + '[ERROR]   ' + Fore.RESET + "Model file does not exist")
            exit(1)

        output_dir = list(filter(None, self.model_dir.split("/")))[-4]
        log_dir = f"{self.model_dir}/log"

        model_log_path = f"{checkpoint_dir}/model_log.pkl"
        if not os.path.exists(model_log_path):
            print(Fore.RED + '[ERROR]   ' + Fore.RESET + "Model log file does not exist")
            exit(1)

        with open(model_log_path, 'rb') as model_log_file:
            model_log = pickle.load(model_log_file)

        model_params = model_log['model_params']
        env_id = model_params['env_id']
        algo_name = model_params['algo']
        seed = model_params['seed']
        n_envs = model_params['n_envs']
        max_episode_steps = model_log['fit_kwargs']['max_episode_steps']
        model_name = model_params['model_name']

        model_log['model_params']['loaded_timestep'] = model_timestep

        algo = eval(algo_name)

        checkpoint_vecnormalize_path = f"{checkpoint_dir}/vecnormalize_{model_timestep}.pkl"
        
        #eval_env = SubprocVecEnv([make_env(env_id, 0, seed, max_episode_steps, config={"render": 1})])
        #eval_env = make_env(env_id, 0, seed, max_episode_steps, config={"render": 1})
        self.eval_env = make_vec_env(env_id, n_envs=1, env_kwargs={"root": self.rootNode})

        self.eval_env = VecNormalize.load(checkpoint_vecnormalize_path, self.eval_env)
        self.eval_env.training = False
        self.eval_env.norm_reward = False
        self.eval_env = VecMonitor(self.eval_env, log_dir)

        self.eval_model = algo.load(checkpoint_path, self.eval_env, tensorboard_log=log_dir)


class RandomActions(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        
        self.rootNode = None
        if kwargs["rootNode"]:
            self.rootNode = kwargs["rootNode"]

        self.env_id = None
        if kwargs["env_id"]:
            self.env_id = kwargs["env_id"]

        self.first_step = True
        self.eval_env = make_vec_env(self.env_id, n_envs=1, env_kwargs={"root": self.rootNode})
        self.observation = None

    def onAnimateBeginEvent(self, dt):
        if self.first_step:
            self.observation = self.eval_env.reset()
            self.first_step = False

        reward, done = self.evaluate()

        return self.observation, reward, done
    
    def evaluate(self):
        action = self.eval_env.action_space.sample()
        print("-------------------------ACTION", action)
        self.observation, reward, done, info = self.eval_env.step([action])

        if done:
            self.observation = self.eval_env.reset()

        return reward, done


class ActionsSequence(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        
        self.rootNode = None
        if kwargs["rootNode"]:
            self.rootNode = kwargs["rootNode"]

        self.env_id = None
        if kwargs["env_id"]:
            self.env_id = kwargs["env_id"]

        self.actions_sequence = None
        if kwargs["actions_sequence"]:
            self.actions_sequence = kwargs["actions_sequence"]

        self.first_step = True
        self.eval_env = make_vec_env(self.env_id, n_envs=1, env_kwargs={"root": self.rootNode})
        self.observation = None
        self.sequence_length = len(self.actions_sequence) - 1
        self.action = 0

    def onEvent(self, event):
        #print(event)

        if event['type'] == "SimulationInitDoneEvent":
            #self.load()
            self.observation = self.eval_env.reset()
            print("-------------------BEGIN SIMULATION")

    def onAnimateBeginEvent(self, dt):
        #obs = self.rootNode.InstrumentCombined.m_ircontroller.xtip.value[0]
        #print("-----------------ANIMATEBEGIN STATE", self.observation, obs)
        
        #if self.first_step:
        #    self.observation = self.eval_env.reset()
        #    self.first_step = False

        #self.eval_env._getState(self.rootNode)
        print("ANIMATE BEGIN: ", getState(self.rootNode))
        
        action = self.actions_sequence[self.action]
        reward, done = self.evaluate(action)
        
        if self.action < self.sequence_length:
            self.action += 1
        else:
            self.action = 0

        return self.observation, reward, done
        
    def evaluate(self, action):
        print("-------------------------ACTION2", action)
        self.observation, reward, done, info = self.eval_env.step([action])
        
        if done:
            self.observation = self.eval_env.reset()
                
        return reward, done
    

    def onKeypressedEvent(self, event):
        key = event['key']

        if key=="F" :
            #obs = self.rootNode.InstrumentCombined.m_ircontroller.xtip.value[0]
            print("-----------------CURRENT STATE", self.observation)

        return 0


class ReloadSim(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.rootNode = None
        if kwargs["rootNode"]:
            self.rootNode = kwargs["rootNode"]

    def onKeypressedEvent(self, event):
        key = event['key']

        if key=="A" :
            self.rootNode.SimRestore.save()

        if key=="M" :
            Sofa.Simulation.reset(self.rootNode)
            obs = self.rootNode.SimRestore.load()
            print("[DEBUG]  RELOADSIM", self.rootNode.InstrumentCombined.m_ircontroller.xtip.value)
            Sofa.Simulation.updateVisual(self.rootNode)

            getState(self.rootNode)
            #self.rootNode.InstrumentCombined.m_ircontroller.init()
            #self.rootNode.InstrumentCombined.m_ircontroller.bwdInit()

        return 0
