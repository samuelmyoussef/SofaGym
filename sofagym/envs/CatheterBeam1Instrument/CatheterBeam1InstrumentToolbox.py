import pathlib
import sys
import os

import pickle

sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute())+"/../")
sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute()))

import numpy as np
import Sofa
import Sofa.Core
import Sofa.Simulation
import SofaRuntime
from splib3.animation.animate import Animation

SofaRuntime.importPlugin("Sofa.Component")


class SimRestore(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.root = kwargs["rootNode"]
        self.data_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'Results/save/data.pckl')
        
    def save(self):
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
        
        goal_pos = _getGoalPos(self.root).tolist()
        
        lengthList = self.root.InstrumentCombined.InterpolGuide.lengthList.value
        #DOF0TransformNode0 = self.root.InstrumentCombined.InterpolGuide.DOF0TransformNode0
        #DOF1TransformNode1 = self.root.InstrumentCombined.InterpolGuide.DOF1TransformNode1.value
        curvAbsList = self.root.InstrumentCombined.InterpolGuide.curvAbsList.value
        edgeList = self.root.InstrumentCombined.InterpolGuide.edgeList.value
        bezier_position = self.root.InstrumentCombined.InterpolGuide.slaves.position.value
        bezier_velocity = self.root.InstrumentCombined.InterpolGuide.slaves.velocity.value

        '''
        try:
            collis_1 = self.root.InstrumentCombined.Collis.LineCollisionModel-LineCollisionModel.getMechanicalState().position.value
            collis_2 = self.root.InstrumentCombined.Collis.LineCollisionModel-PointCollisionModel.getMechanicalState().position.value
            print("--------------------------------------------------------COLLIS")
        except:
            print("NO COLLIS")
        '''

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
                #DOF0TransformNode0,
                #DOF1TransformNode1,
                curvAbsList,
                edgeList,
                bezier_position,
                bezier_velocity,
                free_position,
                free_velocity
                ]
        
        with open(self.data_file, 'wb') as f:
            pickle.dump(data, f)
        
        #print("------------------------SAVED:",  position[-1])
        print("------------------------SAVE")

    def load(self):
        if os.path.exists(self.data_file):
            with open(self.data_file, 'rb') as f:
                loaded_position, loaded_velocity, loaded_derivX, loaded_xtip, loaded_rotation, loaded_indexFirstNode, loaded_activatedPointsBuf, loaded_nodeCurvAbs, loaded_idInstrumentCurvAbsTable, loaded_collis, loaded_goal, loaded_lengthList, loaded_curvAbsList, loaded_edgeList, loaded_bezier_position, loaded_bezier_velocity, loaded_free_position, loaded_free_velocity = pickle.load(f)
            
            self.root.InstrumentCombined.DOFs.position.value = loaded_position
            if np.all(self.root.InstrumentCombined.DOFs.position.value == loaded_position):
                print("[DEBUG]     EQUAL position")
                
            self.root.InstrumentCombined.DOFs.velocity.value = loaded_velocity
            if np.all(self.root.InstrumentCombined.DOFs.velocity.value == loaded_velocity):
                print("[DEBUG]     EQUAL velocity")
    
            self.root.InstrumentCombined.DOFs.free_position.value = loaded_free_position
            if np.all(self.root.InstrumentCombined.DOFs.free_position.value == loaded_free_position):
                print("[DEBUG]     EQUAL free_position")

            self.root.InstrumentCombined.DOFs.free_velocity.value = loaded_free_velocity
            if np.all(self.root.InstrumentCombined.DOFs.free_velocity.value == loaded_free_velocity):
                print("[DEBUG]     EQUAL free_velocity")

            self.root.InstrumentCombined.DOFs.derivX.value = loaded_derivX
            if np.all(self.root.InstrumentCombined.DOFs.derivX.value == loaded_derivX):
                print("[DEBUG]     EQUAL derivX")
            
            self.root.InstrumentCombined.m_ircontroller.xtip.value = loaded_xtip
            if np.all(self.root.InstrumentCombined.m_ircontroller.xtip.value == loaded_xtip):
                print("[DEBUG]     EQUAL xtip")

            self.root.InstrumentCombined.m_ircontroller.rotationInstrument.value = loaded_rotation
            if np.all(self.root.InstrumentCombined.m_ircontroller.rotationInstrument.value == loaded_rotation):
                print("[DEBUG]     EQUAL rotationInstrument")
            
            self.root.InstrumentCombined.m_ircontroller.indexFirstNode.value = loaded_indexFirstNode
            if np.all(self.root.InstrumentCombined.m_ircontroller.indexFirstNode.value == loaded_indexFirstNode):
                print("[DEBUG]     EQUAL indexFirstNode")

            self.root.InstrumentCombined.m_ircontroller.nodeCurvAbs.value = loaded_nodeCurvAbs
            if np.all(self.root.InstrumentCombined.m_ircontroller.nodeCurvAbs.value == loaded_nodeCurvAbs):
                print("[DEBUG]     EQUAL nodeCurvAbs")

            self.root.InstrumentCombined.Collis.CollisionDOFs.position.value = loaded_collis
            if np.all(self.root.InstrumentCombined.Collis.CollisionDOFs.position.value == loaded_collis):
                print("[DEBUG]     EQUAL CollisionDOFs")
            
            with self.root.Goal.GoalMO.position.writeable() as goal:
                goal[0] = loaded_goal

            self.root.InstrumentCombined.InterpolGuide.lengthList.value = loaded_lengthList
            if np.all(self.root.InstrumentCombined.InterpolGuide.lengthList.value == loaded_lengthList):
                print("[DEBUG]     EQUAL lengthList")
            
            self.root.InstrumentCombined.InterpolGuide.curvAbsList.value = loaded_curvAbsList
            if np.all(self.root.InstrumentCombined.InterpolGuide.curvAbsList.value == loaded_curvAbsList):
                print("[DEBUG]     EQUAL curvAbsList")

            self.root.InstrumentCombined.InterpolGuide.edgeList.value = loaded_edgeList
            if np.all(self.root.InstrumentCombined.InterpolGuide.edgeList.value == loaded_edgeList):
                print("[DEBUG]     EQUAL edgeList")
                            
            self.root.InstrumentCombined.InterpolGuide.slaves.position.value = loaded_bezier_position
            if np.all(self.root.InstrumentCombined.InterpolGuide.slaves.position.value == loaded_bezier_position):
                print("[DEBUG]     EQUAL bezier_position")

            self.root.InstrumentCombined.InterpolGuide.slaves.velocity.value = loaded_bezier_velocity
            if np.all(self.root.InstrumentCombined.InterpolGuide.slaves.velocity.value == loaded_bezier_velocity):
                print("[DEBUG]     EQUAL bezier_velocity")

            #with self.root.InstrumentCombined.m_ircontroller.idInstrumentCurvAbsTable.writeable() as idInstrumentCurvAbsTable:
            #    idInstrumentCurvAbsTable = loaded_idInstrumentCurvAbsTable
            
            #print("[DEBUG]      idInstrumentCurvAbsTable", self.root.InstrumentCombined.m_ircontroller.idInstrumentCurvAbsTable.value, "loaded", loaded_idInstrumentCurvAbsTable)

            #with self.root.InstrumentCombined.m_ircontroller.activatedPointsBuf.writeable() as activatedPointsBuf:
            #    activatedPointsBuf = loaded_activatedPointsBuf
                
            #with self.root.InstrumentCombined.InterpolGuide.DOF0TransformNode0.writeable() as DOF0TransformNode0:
            #    DOF0TransformNode0 = loaded_DOF0TransformNode0

            #with self.root.InstrumentCombined.InterpolGuide.DOF1TransformNode1.writeable() as DOF1TransformNode1:
            #    DOF1TransformNode1 = loaded_DOF1TransformNode1

            print("------------------------LOAD")

        obs = np.array(getState(self.root), dtype=np.float32)

        return obs


class RewardShaper(Sofa.Core.Controller):
    """Compute the reward.

    Methods:
    -------
        __init__: Initialization of all arguments.
        getReward: Compute the reward.
        update: Initialize the value of cost.

    Arguments:
    ---------
        rootNode: <Sofa.Core>
            The scene.
        goal_pos: coordinates
            The position of the goal.
        effMO: <MechanicalObject>
            The mechanical object of the element to move.
        cost:
            Evolution of the distance between object and goal.

    """
    def __init__(self, *args, **kwargs):
        """Initialization of all arguments.

        Parameters:
        ----------
            kwargs: Dictionary
                Initialization of the arguments.

        Returns:
        -------
            None.

        """
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.rootNode = None
        if kwargs["rootNode"]:
            self.root = kwargs["rootNode"]
        self.goal_pos = None
        if kwargs["goalPos"]:
            self.goal_pos = kwargs["goalPos"]

        self.init_dist = None
        self.prev_dist = None

    def getReward(self):
        """Compute the reward.

        Parameters:
        ----------
            None.

        Returns:
        -------
            The reward and the cost.

        """
        tip = self.root.InstrumentCombined.DOFs.position[-1][:3]
        current_dist = np.linalg.norm(np.array(tip)-np.array(self.goal_pos))

        return -current_dist, current_dist

    def update(self, goal):
        """Update function.

        This function is used as an initialization function.

        Parameters:
        ----------
            None.

        Arguments:
        ---------
            None.

        """
        self.goal_pos = goal
        tip = self.root.InstrumentCombined.DOFs.position[-1][:3]
        self.init_dist = np.linalg.norm(np.array(tip)-np.array(self.goal_pos))
        self.prev_dist = self.init_dist


class GoalSetter(Sofa.Core.Controller):
    """Compute the goal.

    Methods:
    -------
        __init__: Initialization of all arguments.
        update: Initialize the value of cost.

    Arguments:
    ---------
        goalMO: <MechanicalObject>
            The mechanical object of the goal.
        goalPos: coordinates
            The coordinates of the goal.

    """
    def __init__(self, *args, **kwargs):
        """Initialization of all arguments.

        Parameters:
        ----------
            kwargs: Dictionary
                Initialization of the arguments.

        Returns:
        -------
            None.

        """
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.rootNode = None
        if kwargs["rootNode"]:
            self.rootNode = kwargs["rootNode"]
        self.goal = None
        if kwargs["goal"]:
            self.goal = kwargs["goal"]
        self.goalPos = None
        if kwargs["goalPos"]:
            self.goalPos = kwargs["goalPos"]

    def update(self, goal):
        """Set the position of the goal.

        This function is used as an initialization function.

        Parameters:
        ----------
            None.

        Arguments:
        ---------
            None.

        """
        self.goalPos = goal
        new_position = self.rootNode.CollisionModel.DOFs1.position.value[self.goalPos][:3]
        with self.goal.GoalMO.position.writeable() as position:
            position[0] = new_position

    def set_mo_pos(self, goal):
        """Modify the goal.

        Not used here.
        """
        pass


def _getGoalPos(root):
    """Get XYZ position of the goal.

    Parameters:
    ----------
        rootNode: <Sofa.Core>
            The scene.

    Returns:
    -------
        The position of the goal.
    """
    return root.Goal.GoalMO.position[0]


def getState(root):
    """Compute the state of the environment/agent.

    Parameters:
    ----------
        rootNode: <Sofa.Core>
            The scene.

    Returns:
    -------
        State: list of float
            The state of the environment/agent.
    """
    xtips = []
    rotations = []

    for instrument in range(1):
        xtips.append(root.InstrumentCombined.m_ircontroller.xtip.value[instrument].tolist())
        rotations.append(root.InstrumentCombined.m_ircontroller.rotationInstrument.value[instrument].tolist())

    tip = root.InstrumentCombined.DOFs.position[-1][:3].tolist()

    goal_pos = _getGoalPos(root).tolist()

    state = xtips + rotations + tip + goal_pos
    print("-----------------------------------STATE", xtips, rotations, tip, goal_pos)

    return state


def getReward(root):
    """Compute the reward using Reward.getReward().

    Parameters:
    ----------
        rootNode: <Sofa.Core>
            The scene.

    Returns:
    -------
        done, reward

    """
    reward, cost = root.Reward.getReward()

    if cost <= 5.0:
        return True, 500

    return False, reward


def get_ircontroller_state(node, instrument=0):
    """
    Get state (translation, rotation) of th Interventional Radiology Controller
    """
    print("----------------------------------GET STATE", node.m_ircontroller.xtip.value[instrument])
    return [float(node.m_ircontroller.xtip.value[instrument]),
            float(node.m_ircontroller.rotationInstrument.value[instrument])]


def startCmd(root, action, duration):
    """Initialize the command from root and action.

    Parameters:
    ----------
        rootNode: <Sofa.Core>
            The scene.
        action: int
            The action.
        duration: float
            Duration of the animation.

    Returns:
    ------
        None.

    """
    scale = int(duration/0.01 + 1)
    controlled_instrument, cmd_translation, cmd_rotation = action_to_command(action, scale)
    print("----------------------------------CMD", controlled_instrument, cmd_translation, cmd_rotation)
    source = get_ircontroller_state(root.InstrumentCombined, instrument=controlled_instrument)
    target_translation = source[0] + cmd_translation
    target = [target_translation if target_translation > 0 else 0.1, source[1] + cmd_rotation]
    print("---------------------------------------TARGET", source, target_translation, target)
    #start_cmd(root, root.InstrumentCombined, source, target, duration, controlled_instrument)

    with root.InstrumentCombined.m_ircontroller.xtip.writeable() as xtip:
        xtip[controlled_instrument] = source[0] + (target[0] - source[0])
    if controlled_instrument == 0:
        with root.InstrumentCombined.m_ircontroller.rotationInstrument.writeable() as rotation:
            rotation[0] = source[1] + (target[1] - source[1])


def start_cmd(rootNode, IRC_node, source, target, duration, instrument=0):
    def execute_animation(controller, anim_source, anim_target, factor, anim_instrument):
        """
        Execute animation on the IRC to go from source to target
        """
        factor = 1
        with controller.xtip.writeable() as xtip:
            xtip[anim_instrument] = anim_source[0] + (anim_target[0] - anim_source[0]) * factor
        if anim_instrument == 0:
            with controller.rotationInstrument.writeable() as rotation:
                rotation[0] = anim_source[1] + (anim_target[1] - anim_source[1]) * factor

    rootNode.AnimationManager.addAnimation(
        Animation(
            onUpdate=execute_animation,
            params={"controller": IRC_node.m_ircontroller,
                    "anim_source": source,
                    "anim_target": target,
                    "anim_instrument": instrument},
            duration=duration, mode="once"))

    return


def action_to_command(action, scale):
    """Link between Gym action (int) and SOFA command (displacement of cables).

    Parameters:
    ----------
        action: int
            The number of the action (Gym).

    Returns:
    -------
        The command (number of the cabl and its displacement).
    """

    '''
    if action == 0:
        controlled_instrument = 0
        cmd_translation = 2.0 * scale / 2.0
        cmd_rotation = 0.0
    elif action == 1:
        controlled_instrument = 0
        cmd_translation = 0.0
        cmd_rotation = 1/15 * scale / 2.0
    elif action == 2:
        controlled_instrument = 0
        cmd_translation = 0.0
        cmd_rotation = -1/15 * scale / 2.0
    elif action == 3:
        controlled_instrument = 0
        cmd_translation = -0.7 * scale / 2.0
        cmd_rotation = 0.0
    '''

    if action == 0:
        controlled_instrument = 0
        cmd_translation = 1
        cmd_rotation = 0.0
    elif action == 1:
        controlled_instrument = 0
        cmd_translation = 0.0
        cmd_rotation = 1
    elif action == 2:
        controlled_instrument = 0
        cmd_translation = 0.0
        cmd_rotation = -1
    elif action == 3:
        controlled_instrument = 0
        cmd_translation = -1
        cmd_rotation = 0.0

    else:
        raise NotImplementedError("Action is not in range 0 - 11")

    return controlled_instrument, cmd_translation, cmd_rotation


def getPos(root):
    """Retun the position of the mechanical object of interest.

    Parameters:
    ----------
        root: <Sofa root>
            The root of the scene.

    Returns:
    -------
        _: list
            The position(s) of the object(s) of the scene.
    """
    guide_xtip = root.InstrumentCombined.m_ircontroller.xtip.value[0].tolist()
    guide_rotation = root.InstrumentCombined.m_ircontroller.rotationInstrument.value[0].tolist()

    tip = root.InstrumentCombined.DOFs.position.value.tolist()
    collis = root.InstrumentCombined.Collis.CollisionDOFs.position.value.tolist()
    
    return [guide_xtip, guide_rotation, tip, collis]


def setPos(root, pos):
    """Set the position of the mechanical object of interest.

    Parameters:
    ----------
        root: <Sofa root>
            The root of the scene.
        pos: list
            The position(s) of the object(s) of the scene.

    Returns:
    -------
        None.

    Note:
    ----
        Don't forget to init the new value of the position.

    """
    guide_xtip, guide_rotation, tip, collis = pos
    
    controller = root.InstrumentCombined.m_ircontroller
    with controller.xtip.writeable() as xtip:
        xtip[0] = np.array(guide_xtip)
    
    with controller.rotationInstrument.writeable() as rotation:
        rotation[0] = np.array(guide_rotation)

    root.InstrumentCombined.DOFs.position.value = np.array(tip)
    root.InstrumentCombined.Collis.CollisionDOFs.position.value = np.array(collis)
