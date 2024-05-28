import os
import sys
from typing import Optional

import numpy as np
from gym import spaces
from sympy import DiagMatrix

from sofagym.AbstractEnv import AbstractEnv, ServerEnv
from sofagym.rpc_server import start_scene

import pickle


class CatheterBeam1InstrumentEnv(AbstractEnv):
    """Sub-class of AbstractEnv, dedicated to the catheter beam scene.

    See the class AbstractEnv for arguments and methods.
    """
    #Setting a default configuration
    path = os.path.dirname(os.path.abspath(__file__))
    metadata = {'render.modes': ['human', 'rgb_array']}
    dim_state = 8
    DEFAULT_CONFIG = {"scene": "CatheterBeam1Instrument",
                      "name": "catheter_beam_1_instrument-v0",
                      "deterministic": True,
                      "source": [-1169.51, 298.574, 257.631],
                      "target": [0, 0, 0],
                      "start_node": None,
                      "scale_factor": 1,
                      "dt": 0.01,
                      "timer_limit": 80,
                      "timeout": 50,
                      "display_size": (1600, 800),
                      "render": 1,
                      "save_data": False,
                      "save_image": False,
                      "save_path": path + "/Results" + "/CatheterBeam",
                      "planning": False,
                      "discrete": False,
                      "start_from_history": None,
                      "python_version": sys.version,
                      "zFar": 4000,
                      "time_before_start": 0,
                      "seed": None,
                      "scale": 30,
                      "rotation": [140.0, 0.0, 0.0],
                      "translation": [0.0, 0.0, 0.0],
                      "goal": True,
                      "goalList": [1226, 1663, 1797, 1544, 2233, 2580, 3214],
                      "nb_actions": 4,
                      "dim_state": dim_state,
                      "randomize_states": False,
                      "init_states": [0] * dim_state,
                      "use_server": False
                      }

    def __init__(self, config = None, root=None, use_server: Optional[bool]=False):
        super().__init__(self.DEFAULT_CONFIG, config, render_mode='rgb_array', root=root)
        self.use_server = self.DEFAULT_CONFIG["use_server"]
        # self.env = ServerEnv(self.DEFAULT_CONFIG, config, root=root) if self.use_server else AbstractEnv(self.DEFAULT_CONFIG, config, render_mode='rgb_array', root=root)
        
        self.initialize_states()

        nb_actions = self.config["nb_actions"]
        self.action_space = spaces.Discrete(nb_actions)
        self.nb_actions = str(nb_actions)

        dim_state = self.config["dim_state"]
        low_coordinates = np.array([-1]*dim_state)
        high_coordinates = np.array([1]*dim_state)
        self.observation_space = spaces.Box(low_coordinates, high_coordinates, dtype=np.float32)

        if self.root is None:
            self.init_root()

        self.data_path = os.path.join(self.path, 'Results/save/nodes/')
        self.data_file = None
        self.config_file = os.path.join(self.path, 'Results/save/config.pckl')

    # called when an attribute is not found:
    # def __getattr__(self, name):
    #     # assume it is implemented by self.instance
    #     return self.env.__getattribute__(name)

    def initialize_states(self):
        if self.config["randomize_states"]:
            self.init_states = self.randomize_init_states()
            self.config.update({'init_states': list(self.init_states)})
        else:
            self.init_states = self.config["init_states"]
    
    def randomize_init_states(self):
        """Randomize initial states.

        Returns:
        -------
            init_states: list
                List of random initial states for the environment.
        
        Note:
        ----
            This method should be implemented according to needed random initialization.
        """
        return self.config["init_states"]

    def reset(self):
        """Reset simulation.
        """
        self.initialize_states()
        
        super().reset()

        if self.use_server:
            obs = start_scene(self.config, self.nb_actions)
            state = np.array(obs['observation'], dtype=np.float32)
        else:
            state = np.array(self._getState(self.root), dtype=np.float32)
        
        return state

    def get_available_actions(self):
        """Gives the actions available in the environment.

        Parameters:
        ----------
            None.

        Returns:
        -------
            list of the action available in the environment.
        """
        if isinstance(self.action_space, spaces.Discrete):
            return range(self.action_space.n)
        else:
            return self.action_space
        
    def save_step(self, node):
        position = self.root.InstrumentCombined.DOFs.position.value
        velocity = self.root.InstrumentCombined.DOFs.velocity.value
        derivX = self.root.InstrumentCombined.DOFs.derivX.value
        xtip = self.root.InstrumentCombined.m_ircontroller.xtip.value
        rotation = self.root.InstrumentCombined.m_ircontroller.rotationInstrument.value
        indexFirstNode = self.root.InstrumentCombined.m_ircontroller.indexFirstNode.value
        activatedPointsBuf = self.root.InstrumentCombined.m_ircontroller.activatedPointsBuf.value
        nodeCurvAbs = self.root.InstrumentCombined.m_ircontroller.nodeCurvAbs.value
        idInstrumentCurvAbsTable = self.root.InstrumentCombined.m_ircontroller.idInstrumentCurvAbsTable.value
        free_position = self.root.InstrumentCombined.DOFs.free_position.value
        free_velocity = self.root.InstrumentCombined.DOFs.free_velocity.value
        collis = self.root.InstrumentCombined.Collis.CollisionDOFs.position.value
        lengthList = self.root.InstrumentCombined.InterpolGuide.lengthList.value
        curvAbsList = self.root.InstrumentCombined.InterpolGuide.curvAbsList.value
        edgeList = self.root.InstrumentCombined.InterpolGuide.edgeList.value
        bezier_position = self.root.InstrumentCombined.InterpolGuide.slaves.position.value
        bezier_velocity = self.root.InstrumentCombined.InterpolGuide.slaves.velocity.value
        goal_pos = self._getGoalPos(self.root).tolist()

        data = [
                position,
                velocity,
                derivX,
                xtip,
                rotation,
                indexFirstNode,
                activatedPointsBuf,
                nodeCurvAbs,
                idInstrumentCurvAbsTable,
                collis,
                goal_pos,
                lengthList,
                curvAbsList,
                edgeList,
                bezier_position,
                bezier_velocity,
                free_position,
                free_velocity
                ]
        
        self.data_file = os.path.join(self.data_path, str(node) + ".pckl")
        with open(self.data_file, 'wb') as f:
            pickle.dump(data, f)

    def load_step(self, data_file):
        if os.path.exists(data_file):
            with open(data_file, 'rb') as f:
                loaded_position, loaded_velocity, loaded_derivX, loaded_xtip, loaded_rotation, loaded_indexFirstNode, loaded_activatedPointsBuf, loaded_nodeCurvAbs, loaded_idInstrumentCurvAbsTable, loaded_collis, loaded_goal, loaded_lengthList, loaded_curvAbsList, loaded_edgeList, loaded_bezier_position, loaded_bezier_velocity, loaded_free_position, loaded_free_velocity = pickle.load(f)
            
            self.root.InstrumentCombined.DOFs.position.value = loaded_position
            self.root.InstrumentCombined.DOFs.velocity.value = loaded_velocity
            self.root.InstrumentCombined.DOFs.free_position.value = loaded_free_position
            self.root.InstrumentCombined.DOFs.free_velocity.value = loaded_free_velocity
            self.root.InstrumentCombined.DOFs.derivX.value = loaded_derivX
            self.root.InstrumentCombined.m_ircontroller.xtip.value = loaded_xtip
            self.root.InstrumentCombined.m_ircontroller.rotationInstrument.value = loaded_rotation
            self.root.InstrumentCombined.m_ircontroller.indexFirstNode.value = loaded_indexFirstNode
            self.root.InstrumentCombined.m_ircontroller.nodeCurvAbs.value = loaded_nodeCurvAbs
            self.root.InstrumentCombined.Collis.CollisionDOFs.position.value = loaded_collis
            self.root.InstrumentCombined.InterpolGuide.lengthList.value = loaded_lengthList
            self.root.InstrumentCombined.InterpolGuide.curvAbsList.value = loaded_curvAbsList
            self.root.InstrumentCombined.InterpolGuide.edgeList.value = loaded_edgeList      
            self.root.InstrumentCombined.InterpolGuide.slaves.position.value = loaded_bezier_position
            self.root.InstrumentCombined.InterpolGuide.slaves.velocity.value = loaded_bezier_velocity
            
            with self.root.Goal.GoalMO.position.writeable() as goal:
                goal[0] = loaded_goal

        obs = np.array(self._getState(self.root), dtype=np.float32)

        return obs

    def save_config(self):
        with open(self.config_file, 'wb') as f:
            pickle.dump(self.config, f)

    def load_config(self, config_file):
        if os.path.exists(config_file):
            with open(config_file, 'rb') as f:
                self.config = pickle.load(f)
        
        self.config_file = config_file
