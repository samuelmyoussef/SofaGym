# -*- coding: utf-8 -*-
"""Create the scene with the Abstraction of Jimmy.


Units: cm, kg, s.
"""

__authors__ = ("emenager")
__contact__ = ("etienne.menager@ens-rennes.fr")
__version__ = "1.0.0"
__copyright__ = "(c) 2021, Inria"
__date__ = "August 12 2021"

VISUALISATION = False

import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute())+"/../")
sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute()))

from sofagym.header import addHeader as header
from sofagym.header import addVisu as Visu

from CartStemContact import CartStem, Contacts
from CartStemContactToolbox import rewardShaper, goalSetter, sceneModerator, applyAction


def add_goal_node(root, pos):
    goal = root.addChild("Goal")
    goal.addObject('MechanicalObject', name='GoalMO', showObject=False)
    goal.addObject('MeshOBJLoader', name="loader", filename='mesh/cylinder.obj', scale3d=[0.05, 3, 0.05],
                   rotation=[90, 0, 0], translation=[pos[0], pos[1], pos[2]-20])
    goal.addObject('OglModel',  name='GoalOgl', src='@loader', color=[1, 0, 0, 0.5])
    return goal


from vedo import Mesh, Plotter
import Sofa

class CartStemContactVis(Sofa.Core.Controller):
    def __init__(self, viewer, root, env=None, *args, **kwargs):
        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.root = root
        self.env = env
        self.goal = self.root.Goal
        self.cart_obj = self.root.cartstem.Cart.Visu.getObject("objOgl")
        self.sphere = self.root.cartstem.Cart.MappedFrames.sphere_VisualModel.getObject("model")
        self.contacts = self.root.contacts
        self.Cube1 = self.contacts.Cube_1.Visu.getObject("objOgl")
        self.Cube2 = self.contacts.Cube_2.Visu.getObject("objOgl")

        self.vedo_viewer = viewer
        self.camera = self.vedo_viewer.camera
        self.vedo_viewer.look_at(plane='xz')
        
        self.vedo_cart_mesh = Mesh()
        self.vedo_goal_mesh = Mesh()

    def onSimulationInitDoneEvent(self, _):
        # Init the Vedo Mesh
        cart_positions = self.cart_obj.position.value
        cart_triangles = self.cart_obj.triangles.value

        sphere_positions = self.sphere.position.value 
        sphere_quads = self.sphere.quads.value

        Cube_1_positions = self.Cube1.position.value
        Cube_1_triangles = self.Cube1.triangles.value
        
        Cube_2_positions = self.Cube2.position.value
        Cube_2_triangles = self.Cube2.triangles.value        

        self.goal = self.root.Goal
        goal_positions = self.goal.getObject('GoalOgl').position.value
        goal_quads = self.goal.getObject('GoalOgl').quads.value

        self.vedo_cart_mesh = Mesh(inputobj=[cart_positions, cart_triangles], c='blue5').wireframe(False).linewidth(1.5)
        self.vedo_sphere_mesh = Mesh(inputobj=[sphere_positions, sphere_quads], c='green5').wireframe(False).linewidth(1.5)
        self.vedo_cube_1_mesh = Mesh(inputobj=[Cube_1_positions, Cube_1_triangles], c='green5').wireframe(False).linewidth(1.5)
        self.vedo_cube_2_mesh = Mesh(inputobj=[Cube_2_positions, Cube_2_triangles], c='green5').wireframe(False).linewidth(1.5)
        self.vedo_goal_mesh = Mesh(inputobj=[goal_positions, goal_quads], c='red5').wireframe(False).linewidth(1.5)
        actors = [self.vedo_cart_mesh, self.vedo_sphere_mesh, self.vedo_cube_1_mesh, self.vedo_cube_2_mesh, self.vedo_goal_mesh]

        # Init the Vedo Viewer
        self.vedo_viewer.add(actors)
        self.vedo_viewer.show(actors, mode=0)

    def onAnimateEndEvent(self, _):
        # Update the Vedo Mesh (can be updated on place)
        cart_positions = self.cart_obj.position.value
        self.vedo_cart_mesh.points(cart_positions)

        sphere_positions = self.sphere.position.value
        self.vedo_sphere_mesh.points(sphere_positions)

        Cube_1_positions = self.Cube1.position.value
        self.vedo_cube_1_mesh.points(Cube_1_positions)

        Cube_2_positions = self.Cube2.position.value
        self.vedo_cube_2_mesh.points(Cube_2_positions)
        
        goal_positions = self.goal.getObject('GoalOgl').position.value
        self.vedo_goal_mesh.points(goal_positions)

        # Update the Vedo Viewer
        self.vedo_viewer.render()



def createScene(rootNode, config={"source": [0, -50, 10],
                                  "target": [0, 0, 10],
                                  "goalPos": [7, 0, 20],
                                  "seed": None,
                                  "zFar": 4000,
                                  "init_x": 0,
                                  "cube_x": [-6, 6],
                                  "max_move": 7.5,
                                  "dt": 0.01},
                         mode='simu_and_visu', viewer=None):
    
    # Choose the mode: visualization or computations (or both)
    visu, simu = False, False
    if 'visu' in mode:
        visu = True
    if 'simu' in mode:
        simu = True

    header(rootNode, alarmDistance=1.0, contactDistance=0.1, tolerance=1e-6, maxIterations=100, gravity=[0,0,-981.0],
           dt=config['dt'])

    position_spot = [[0, -50, 10]]
    direction_spot = [[0.0, 1, 0]]
    Visu(rootNode, config, position_spot, direction_spot, cutoff=250)

    max_move = config['max_move']
    assert config['cube_x'][0] < config['cube_x'][1]
    bound = [config['cube_x'][0]+3, config['cube_x'][1]-3]
    init_x = max(-min(config["init_x"], bound[1]), bound[0])

    max_v = 2
    cosserat_config = {'init_pos': [init_x, 0, 0], 'tot_length': 25, 'nbSectionS': 1, 'nbFramesF': 20}
    cartstem_config = {"init_pos": [init_x, 0, 0], "cart_size": [2, 2, 5], "max_move": max_move,  "max_v": max_v,
                       "dt": config["dt"],  "cosserat_config": cosserat_config}
    contact_config = {"init_pos": [0, 0, 12], "cube_size": [2, 1, 2], "cube_x": config["cube_x"]}

    cartstem = CartStem(cartstem_config=cartstem_config)
    cartstem.onEnd(rootNode)

    contacts = Contacts(contact_config=contact_config)
    contacts.onEnd(rootNode)

    add_goal_node(rootNode, config["goalPos"])

    rootNode.addObject(goalSetter(name="GoalSetter", goalPos=config["goalPos"]))
    rootNode.addObject(rewardShaper(name="Reward", rootNode=rootNode, max_dist=cartstem_config['max_move']))
    rootNode.addObject(sceneModerator(name="sceneModerator",  cartstem=cartstem, contacts=contacts))
    rootNode.addObject(applyAction(name="applyAction", root=rootNode, cartstem=cartstem))

    if visu:
        cart_visu = rootNode.addObject(CartStemContactVis(viewer=viewer, root=rootNode, name="CartStemContact Visualization") )

    return rootNode
