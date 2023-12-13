import math
import os
import sys
from typing import Optional

import numpy as np
from gym import spaces
from sofagym.AbstractEnv import AbstractEnv, ServerEnv
from sofagym.rpc_server import start_scene


class CartPoleEnv:
    """Sub-class of AbstractEnv, dedicated to the cart pole scene.

    See the class AbstractEnv for arguments and methods.
    """
    #Setting a default configuration
    path = path = os.path.dirname(os.path.abspath(__file__))
    metadata = {'render.modes': ['human', 'rgb_array']}
    dim_state = 4
    DEFAULT_CONFIG = {"scene": "CartPole",
                      "deterministic": True,
                      "source": [0, 0, 160],
                      "target": [0, 0, 0],
                      "goal": False,
                      "start_node": None,
                      "scale_factor": 10,
                      "dt": 0.001,
                      "timer_limit": 80,
                      "timeout": 50,
                      "display_size": (1600, 800),
                      "render": 0,
                      "save_data": False,
                      "save_image": False,
                      "save_path": path + "/Results" + "/CartPole",
                      "planning": False,
                      "discrete": False,
                      "start_from_history": None,
                      "python_version": sys.version,
                      "zFar": 4000,
                      "time_before_start": 0,
                      "seed": None,
                      "nb_actions": 2,
                      "dim_state": dim_state,
                      "init_x": 0,
                      "x_threshold": 100,
                      "max_move": 24,
                      "max_angle": 0.418,
                      "randomize_states": True,
                      "init_states": [0] * dim_state,
                      "use_server": False
                      }

    def __init__(self, config=None, root=None, use_server: Optional[bool]=False):
        self.use_server = self.DEFAULT_CONFIG["use_server"]
        self.env = ServerEnv(self.DEFAULT_CONFIG, config, root=root) if self.use_server else AbstractEnv(self.DEFAULT_CONFIG, config, root=root)
        print("----------------------------------ROOOOOOOOT", type(root))

        self.init_states = self.env.config["init_states"]
        if self.env.config["randomize_states"]:
            self.randomize_init_states()

        self.x_threshold = self.env.config["x_threshold"]
        self.theta_threshold_radians = self.env.config["max_move"] * math.pi / 180
        self.env.config.update({'max_angle': self.theta_threshold_radians})
        
        high = np.array(
            [
                self.x_threshold * 2,
                np.finfo(np.float32).max,
                self.theta_threshold_radians,
                np.finfo(np.float32).max,
            ],
            dtype=np.float32,
        )

        nb_actions = self.env.config["nb_actions"]
        self.env.action_space = spaces.Discrete(nb_actions)
        self.nb_actions = str(nb_actions)

        self.env.observation_space = spaces.Box(-high, high, dtype=np.float32)

    # called when an attribute is not found:
    def __getattr__(self, name):
        # assume it is implemented by self.instance
        return self.env.__getattribute__(name)


    def randomize_init_states(self):
        self.init_states = self.env.np_random.uniform(low=-0.05, high=0.05, size=(self.env.config["dim_state"],))
        print("--------------------------------INIT STATE", self.init_states)
        self.env.config.update({'init_states': list(self.init_states)})


    def reset(self):
        """Reset simulation.
        """
        if self.env.config["randomize_states"]:
            self.randomize_init_states()
        
        self.env.reset()

        init_states = self.env.np_random.uniform(low=-0.05, high=0.05, size=(4,))
        print("--------------------------------INIT STATE", init_states)
        self.env.config.update({'init_states': list(init_states)})
        
        if self.use_server:
            obs = start_scene(self.env.config, self.nb_actions)
            return np.array(obs['observation'])
        
        return np.array(init_states)

    def get_available_actions(self):
        """Gives the actions available in the environment.

        Parameters:
        ----------
            None.

        Returns:
        -------
            list of the action available in the environment.
        """
        return self.env.action_space
