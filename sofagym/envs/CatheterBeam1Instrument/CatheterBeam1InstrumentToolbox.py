import pathlib
import sys
import os

import pickle

import math

sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute())+"/../")
sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute()))

from networkx import current_flow_betweenness_centrality
import numpy as np
import Sofa
import Sofa.Core
import Sofa.Simulation
import SofaRuntime
from splib3.animation.animate import Animation

SofaRuntime.importPlugin("Sofa.Component")




from collections import defaultdict

import SofaRuntime

class Graph:
    def __init__(self):
        """
        self.edges is a dict of all possible next nodes
        e.g. {'X': ['A', 'B', 'C', 'E'], ...}
        self.weights has all the weights between two nodes,
        with the two nodes as a tuple as the key
        e.g. {('X', 'A'): 7, ('X', 'B'): 2, ...}
        """
        self.edges = defaultdict(list)
        self.weights = {}

    def add_edge(self, from_node, to_node, weight):
        # Note: assumes edges are bi-directional
        self.edges[from_node].append(to_node)
        self.edges[to_node].append(from_node)
        self.weights[(from_node, to_node)] = weight
        self.weights[(to_node, from_node)] = weight


def dijkstra(graph, initial, end):
    # shortest paths is a dict of nodes
    # whose value is a tuple of (previous node, weight)
    shortest_paths = {initial: (None, 0)}
    current_node = initial
    visited = set()

    while current_node != end:
        visited.add(current_node)
        destinations = graph.edges[current_node]
        weight_to_current_node = shortest_paths[current_node][1]

        for next_node in destinations:
            weight = graph.weights[(current_node, next_node)] + weight_to_current_node
            if next_node not in shortest_paths:
                shortest_paths[next_node] = (current_node, weight)
            else:
                current_shortest_weight = shortest_paths[next_node][1]
                if current_shortest_weight > weight:
                    shortest_paths[next_node] = (current_node, weight)

        next_destinations = {node: shortest_paths[node] for node in shortest_paths if node not in visited}
        if not next_destinations:
            return "Route Not Possible"
        # next node is the destination with the lowest weight
        current_node = min(next_destinations, key=lambda k: next_destinations[k][1])

    # Work back through destinations in shortest path
    graph_path = []
    while current_node is not None:
        graph_path.append(current_node)
        next_node = shortest_paths[current_node][0]
        current_node = next_node

    # Reverse path
    graph_path = graph_path[::-1]
    graph_length = [shortest_paths[k][1] for k in graph_path]
    return graph_path, graph_length




# class SimRestore(Sofa.Core.Controller):
#     def __init__(self, *args, **kwargs):
#         Sofa.Core.Controller.__init__(self, *args, **kwargs)

#         self.root = kwargs["rootNode"]
#         self.data_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'Results/save/data.pckl')
        
#     def save(self):
#         position = self.root.InstrumentCombined.DOFs.position.value
#         velocity = self.root.InstrumentCombined.DOFs.velocity.value
#         derivX = self.root.InstrumentCombined.DOFs.derivX.value
#         xtip = self.root.InstrumentCombined.m_ircontroller.xtip.value
#         rotation = self.root.InstrumentCombined.m_ircontroller.rotationInstrument.value
#         indexFirstNode = self.root.InstrumentCombined.m_ircontroller.indexFirstNode.value
#         activatedPointsBuf = self.root.InstrumentCombined.m_ircontroller.activatedPointsBuf.value
#         nodeCurvAbs = self.root.InstrumentCombined.m_ircontroller.nodeCurvAbs.value
#         idInstrumentCurvAbsTable = self.root.InstrumentCombined.m_ircontroller.idInstrumentCurvAbsTable.value
#         free_position = self.root.InstrumentCombined.DOFs.free_position.value
#         free_velocity = self.root.InstrumentCombined.DOFs.free_velocity.value

#         collis = self.root.InstrumentCombined.Collis.CollisionDOFs.position.value
        
#         goal_pos = _getGoalPos(self.root).tolist()
        
#         lengthList = self.root.InstrumentCombined.InterpolGuide.lengthList.value
#         #DOF0TransformNode0 = self.root.InstrumentCombined.InterpolGuide.DOF0TransformNode0
#         #DOF1TransformNode1 = self.root.InstrumentCombined.InterpolGuide.DOF1TransformNode1.value
#         curvAbsList = self.root.InstrumentCombined.InterpolGuide.curvAbsList.value
#         edgeList = self.root.InstrumentCombined.InterpolGuide.edgeList.value
#         bezier_position = self.root.InstrumentCombined.InterpolGuide.slaves.position.value
#         bezier_velocity = self.root.InstrumentCombined.InterpolGuide.slaves.velocity.value

#         '''
#         try:
#             collis_1 = self.root.InstrumentCombined.Collis.LineCollisionModel-LineCollisionModel.getMechanicalState().position.value
#             collis_2 = self.root.InstrumentCombined.Collis.LineCollisionModel-PointCollisionModel.getMechanicalState().position.value
#             print("--------------------------------------------------------COLLIS")
#         except:
#             print("NO COLLIS")
#         '''

#         data = [
#                 position,
#                 velocity,
#                 derivX,
#                 xtip,
#                 rotation,
#                 indexFirstNode,
#                 activatedPointsBuf,
#                 nodeCurvAbs,
#                 idInstrumentCurvAbsTable,
#                 collis,
#                 goal_pos,
#                 lengthList,
#                 #DOF0TransformNode0,
#                 #DOF1TransformNode1,
#                 curvAbsList,
#                 edgeList,
#                 bezier_position,
#                 bezier_velocity,
#                 free_position,
#                 free_velocity
#                 ]
        
#         with open(self.data_file, 'wb') as f:
#             pickle.dump(data, f)
        
#         #print("------------------------SAVED:",  position[-1])
#         print("------------------------SAVE")

#     def load(self):
#         if os.path.exists(self.data_file):
#             with open(self.data_file, 'rb') as f:
#                 loaded_position, loaded_velocity, loaded_derivX, loaded_xtip, loaded_rotation, loaded_indexFirstNode, loaded_activatedPointsBuf, loaded_nodeCurvAbs, loaded_idInstrumentCurvAbsTable, loaded_collis, loaded_goal, loaded_lengthList, loaded_curvAbsList, loaded_edgeList, loaded_bezier_position, loaded_bezier_velocity, loaded_free_position, loaded_free_velocity = pickle.load(f)
            
#             self.root.InstrumentCombined.DOFs.position.value = loaded_position
#             if np.all(self.root.InstrumentCombined.DOFs.position.value == loaded_position):
#                 print("[DEBUG]     EQUAL position")
                
#             self.root.InstrumentCombined.DOFs.velocity.value = loaded_velocity
#             if np.all(self.root.InstrumentCombined.DOFs.velocity.value == loaded_velocity):
#                 print("[DEBUG]     EQUAL velocity")
    
#             self.root.InstrumentCombined.DOFs.free_position.value = loaded_free_position
#             if np.all(self.root.InstrumentCombined.DOFs.free_position.value == loaded_free_position):
#                 print("[DEBUG]     EQUAL free_position")

#             self.root.InstrumentCombined.DOFs.free_velocity.value = loaded_free_velocity
#             if np.all(self.root.InstrumentCombined.DOFs.free_velocity.value == loaded_free_velocity):
#                 print("[DEBUG]     EQUAL free_velocity")

#             self.root.InstrumentCombined.DOFs.derivX.value = loaded_derivX
#             if np.all(self.root.InstrumentCombined.DOFs.derivX.value == loaded_derivX):
#                 print("[DEBUG]     EQUAL derivX")
            
#             self.root.InstrumentCombined.m_ircontroller.xtip.value = loaded_xtip
#             if np.all(self.root.InstrumentCombined.m_ircontroller.xtip.value == loaded_xtip):
#                 print("[DEBUG]     EQUAL xtip")

#             self.root.InstrumentCombined.m_ircontroller.rotationInstrument.value = loaded_rotation
#             if np.all(self.root.InstrumentCombined.m_ircontroller.rotationInstrument.value == loaded_rotation):
#                 print("[DEBUG]     EQUAL rotationInstrument")
            
#             self.root.InstrumentCombined.m_ircontroller.indexFirstNode.value = loaded_indexFirstNode
#             if np.all(self.root.InstrumentCombined.m_ircontroller.indexFirstNode.value == loaded_indexFirstNode):
#                 print("[DEBUG]     EQUAL indexFirstNode")

#             self.root.InstrumentCombined.m_ircontroller.nodeCurvAbs.value = loaded_nodeCurvAbs
#             if np.all(self.root.InstrumentCombined.m_ircontroller.nodeCurvAbs.value == loaded_nodeCurvAbs):
#                 print("[DEBUG]     EQUAL nodeCurvAbs")

#             self.root.InstrumentCombined.Collis.CollisionDOFs.position.value = loaded_collis
#             if np.all(self.root.InstrumentCombined.Collis.CollisionDOFs.position.value == loaded_collis):
#                 print("[DEBUG]     EQUAL CollisionDOFs")
            
#             with self.root.Goal.GoalMO.position.writeable() as goal:
#                 goal[0] = loaded_goal

#             self.root.InstrumentCombined.InterpolGuide.lengthList.value = loaded_lengthList
#             if np.all(self.root.InstrumentCombined.InterpolGuide.lengthList.value == loaded_lengthList):
#                 print("[DEBUG]     EQUAL lengthList")
            
#             self.root.InstrumentCombined.InterpolGuide.curvAbsList.value = loaded_curvAbsList
#             if np.all(self.root.InstrumentCombined.InterpolGuide.curvAbsList.value == loaded_curvAbsList):
#                 print("[DEBUG]     EQUAL curvAbsList")

#             self.root.InstrumentCombined.InterpolGuide.edgeList.value = loaded_edgeList
#             if np.all(self.root.InstrumentCombined.InterpolGuide.edgeList.value == loaded_edgeList):
#                 print("[DEBUG]     EQUAL edgeList")
                            
#             self.root.InstrumentCombined.InterpolGuide.slaves.position.value = loaded_bezier_position
#             if np.all(self.root.InstrumentCombined.InterpolGuide.slaves.position.value == loaded_bezier_position):
#                 print("[DEBUG]     EQUAL bezier_position")

#             self.root.InstrumentCombined.InterpolGuide.slaves.velocity.value = loaded_bezier_velocity
#             if np.all(self.root.InstrumentCombined.InterpolGuide.slaves.velocity.value == loaded_bezier_velocity):
#                 print("[DEBUG]     EQUAL bezier_velocity")

#             #with self.root.InstrumentCombined.m_ircontroller.idInstrumentCurvAbsTable.writeable() as idInstrumentCurvAbsTable:
#             #    idInstrumentCurvAbsTable = loaded_idInstrumentCurvAbsTable
            
#             #print("[DEBUG]      idInstrumentCurvAbsTable", self.root.InstrumentCombined.m_ircontroller.idInstrumentCurvAbsTable.value, "loaded", loaded_idInstrumentCurvAbsTable)

#             #with self.root.InstrumentCombined.m_ircontroller.activatedPointsBuf.writeable() as activatedPointsBuf:
#             #    activatedPointsBuf = loaded_activatedPointsBuf
                
#             #with self.root.InstrumentCombined.InterpolGuide.DOF0TransformNode0.writeable() as DOF0TransformNode0:
#             #    DOF0TransformNode0 = loaded_DOF0TransformNode0

#             #with self.root.InstrumentCombined.InterpolGuide.DOF1TransformNode1.writeable() as DOF1TransformNode1:
#             #    DOF1TransformNode1 = loaded_DOF1TransformNode1

#             print("------------------------LOAD")

#         obs = np.array(getState(self.root), dtype=np.float32)

#         return obs


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

        self.root = None
        if kwargs["rootNode"]:
            self.root = kwargs["rootNode"]
        self.goal_pos = None
        if kwargs["goalPos"]:
            self.goal_pos = kwargs["goalPos"]

        self.init_dist = None
        self.prev_dist = None

        self.path_mesh = None
        if kwargs["path_mesh"]:
            self.path_mesh = kwargs["path_mesh"]
        self.path_mo = None
        if kwargs["path_mo"]:
            self.path_mo = kwargs["path_mo"]
        self.tip_mo = None
        if kwargs["tip_mo"]:
            self.tip_mo = kwargs["tip_mo"]

        self.shortest_path_mo = None
        if kwargs["shortest_path_mo"]:
            self.shortest_path_mo = kwargs["shortest_path_mo"]

        self.projection_mo = None
        if kwargs["projection_mo"]:
            self.projection_mo = kwargs["projection_mo"]

        self.prev_ratio = 0.0
        self.prev_dist = None

        self.prev_start = None
        self.proj = None

    def getReward(self):
        """Compute the reward.

        Parameters:
        ----------
            None.

        Returns:
        -------
            The reward and the cost.

        """
        # tip = self.root.InstrumentCombined.DOFs.position[-1][:3]
        # current_dist = np.linalg.norm(np.array(tip)-np.array(self.goal_pos))

        # # current_dist_norm = self.normalize_reward(current_dist)
        # # print("DEBUG    getREWARD", current_dist_norm, current_dist)

        # return -current_dist, current_dist
    
        # pos = self.tip_mo.position[-1][:3]
        # dist_to_path = [{'id': k, 'dist': np.linalg.norm(pos[:] - path_point[:])}
        #                 for k, path_point in enumerate(self.path_pos)]
        # sorted_dist = sorted(dist_to_path, key=lambda item: item['dist'])
        # if len(sorted_dist) < 1:
        #     return 0.0
        # closest_points = [sorted_dist[0]["id"], sorted_dist[1]["id"]]

        # print("DEBUG clo", closest_points)

        pos = np.asarray(self.tip_mo.position[-1][:3])
        b, i = self.update_path_dist(pos)
        a = self.prev_start

        self.proj = self.calculate_tip_projection(a, b, pos)

        # a = np.asarray(self.path_pos[closest_points[0]])
        # b = np.asarray(self.path_pos[closest_points[1]])

        # with self.shortest_path_mo.position.writeable() as path_mo:
        #     path_mo[0] = self.path_pos[closest_points[1]][:]

        # ap = pos - a
        # ab = b - a

        # proj = a + np.dot(ap, ab)/np.dot(ab,ab) * ab

        # with self.projection_mo.position.writeable() as proj_mo:
        #     proj_mo[0] = proj[:]

        # print("DEBUG   proj", a, b, ap, ab, proj)

        new_dist = np.linalg.norm(self.proj - b)

        print(f"DEBUG   proj: {self.proj}   b: {b}  dist: {new_dist}")

        goal = self.root.Goal.GoalMO.position[0]
        goal_dist = np.linalg.norm(pos - goal)

        # ratio = min(1/new_dist, 1)

        ratio = max(((self.prev_dist - new_dist) / self.prev_dist), 0)

        sigmoid = 1 / (1 + math.exp(-ratio))

        self.prev_dist = new_dist

        if new_dist <= 1:
            self.prev_start = pos
            self.path_pos.pop(i)

            b, i = self.update_path_dist(pos)

        
        # if new_dist <= 1:
        #     new_closest_point = sorted_dist[1]["id"]
            
        #     self.prev_dist = dist_to_path[new_closest_point]["dist"]
        #     self.prev_ratio = 0.0

        #     with self.shortest_path_mo.position.writeable() as path_mo:
        #         path_mo[0] = self.path_pos[new_closest_point][:]

        #     print("Checkpoint Reached")
        #     print("DEBUG    current target", self.path_pos[new_closest_point][:])

        #     self.path_pos.pop(closest_point)
        
        # # elif ratio > self.prev_ratio:
        #     # self.prev_ratio = ratio
        # else:
        #     self.prev_dist = new_dist

        return sigmoid, None
    
    def normalize_reward(self, dist):
        min_dist = -5000
        max_dist = 0
        dist_range = max_dist - min_dist

        dist_norm = (dist - abs(min_dist))/ dist_range

        return abs(dist_norm)

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
        # self.goal_pos = goal
        # tip = self.root.InstrumentCombined.DOFs.position[-1][:3]
        # self.init_dist = np.linalg.norm(np.array(tip)-np.array(self.goal_pos))
        # self.prev_dist = self.init_dist
        
        self.goal_pos = goal
        
        edges = []
        with self.path_mesh.edges.writeable() as Topoedges:
            for edge in Topoedges:
                edges += [(edge[0], edge[1], np.linalg.norm(np.array(self.path_mesh.position.value[edge[0]]) -
                                                            np.array(self.path_mesh.position.value[edge[1]])))]

        self.path_graph = Graph()
        for edge in edges:
            self.path_graph.add_edge(*edge)

        self.path, self.path_length = dijkstra(self.path_graph, 263, self.goal_pos)
        # path2, path_length2 = dijkstra(self.path_graph, 264, self.goal_pos)
        # if len(path1) > len(path2):
        #     self.path, self.path_length = path1, path_length1
        # else:
        #     self.path, self.path_length = path2, path_length2
        self.path_pos = []
        for point in self.path:
            self.path_pos.append(self.path_mo.position.value[point][:3].tolist())
        
        # shortest_path = self.root.addChild("Dijkstra")
        # shortest_path.addObject('VisualStyle', displayFlags="showCollisionModels")
        # shortest_path_mo = shortest_path.addObject('MechanicalObject', name='PathMO', showObject=True, drawMode="1", showObjectScale=1.0,
        #                      showColor=[0, 1, 0, 0.5], position=self.path_pos)
        # print("DEBUG        path_pos", self.path_pos)
        # print("DEBUG        p_mo", self.path_mesh.position.value)

        self.shortest_path_mo.position.value = self.path_pos

        # pos = self.tip_mo.position[-1][:3]
        # dist_to_path = [{'id': k, 'dist': np.linalg.norm(pos - path_point)}
        #                 for k, path_point in enumerate(self.path_pos)]
        # sorted_dist = sorted(dist_to_path, key=lambda item: item['dist'])
        # if len(sorted_dist) < 1:
        #     return 0.0
        # closest_point = sorted_dist[0]["id"]
        
        # self.prev_dist = dist_to_path[closest_point]["dist"]
        # self.prev_ratio = 0.0

        pos = np.asarray(self.tip_mo.position[-1][:3])
        a = pos
        b, i = self.update_path_dist(pos)

        self.proj = self.calculate_tip_projection(a, b, pos)
        self.prev_start = a

        self.prev_dist = np.linalg.norm(self.proj - b)

    def update_path_dist(self, pos):
        # pos = self.tip_mo.position[-1][:3]
        
        dist_to_path = [{'id': k, 'dist': np.linalg.norm(pos - path_point)}
                        for k, path_point in enumerate(self.path_pos)]
        sorted_dist = sorted(dist_to_path, key=lambda item: item['dist'])
        if len(sorted_dist) < 1:
            return 0.0
        
        closest_point = sorted_dist[0]["id"]

        self.prev_dist = dist_to_path[closest_point]["dist"]

        with self.shortest_path_mo.position.writeable() as path_mo:
            path_mo[0] = self.path_pos[closest_point][:]

        return np.asarray(self.path_pos[closest_point]), closest_point


    def calculate_tip_projection(self, a, b, p):
        ap = p - a
        ab = b - a

        proj = a + np.dot(ap, ab)/np.dot(ab,ab) * ab

        with self.projection_mo.position.writeable() as proj_mo:
            proj_mo[0] = proj[:]
        
        return proj


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
        new_position = self.rootNode.CollisionModel.Centerline.dofs.position.value[self.goalPos][:3]
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
    # print("-----------------------------------STATE", xtips, rotations, tip, goal_pos)

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
    # reward, cost = root.Reward.getReward()

    # if cost <= 5.0:
    #     return True, 500

    # return False, reward

    goal_radius = 5
    done = False

    goal = root.Goal.GoalMO.position[0]
    tip_pos = root.InstrumentCombined.DOFs.position[-1][:3]
    dist = np.linalg.norm(tip_pos - goal)

    reward, cost = root.Reward.getReward()
    
    if dist < goal_radius:
        done = True

    return done, reward


def get_ircontroller_state(node, instrument=0):
    """
    Get state (translation, rotation) of th Interventional Radiology Controller
    """
    # print("----------------------------------GET STATE", node.m_ircontroller.xtip.value[instrument])
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
    # print("----------------------------------CMD", controlled_instrument, cmd_translation, cmd_rotation)
    source = get_ircontroller_state(root.InstrumentCombined, instrument=controlled_instrument)
    target_translation = source[0] + cmd_translation
    target = [target_translation if target_translation > 0 else 0.1, source[1] + cmd_rotation]
    # print("---------------------------------------TARGET", source, target_translation, target)
    start_cmd(root, root.InstrumentCombined, source, target, duration, controlled_instrument)

    '''with root.InstrumentCombined.m_ircontroller.xtip.writeable() as xtip:
        xtip[controlled_instrument] = source[0] + (target[0] - source[0])
    if controlled_instrument == 0:
        with root.InstrumentCombined.m_ircontroller.rotationInstrument.writeable() as rotation:
            rotation[0] = source[1] + (target[1] - source[1])'''


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
