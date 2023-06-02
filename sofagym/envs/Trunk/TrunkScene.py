import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute())+"/../")
sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute()))


from splib3.animation import AnimationManagerController
from math import cos, sin
import numpy as np
from splib3.objectmodel import SofaPrefab, SofaObject
from splib3.numerics import Vec3, Quat


from TrunkToolbox import rewardShaper, goalSetter

import os
path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'

from numpy import zeros, array
from numpy.random import uniform
from numpy.linalg import norm
import Sofa
from SSD.SOFA import UserAPI, Database
from vedo import Mesh, Plotter




def add_goal_node(root, pos):
    '''
    goal = root.addChild("Goal")
    goal.addObject('VisualStyle', displayFlags="showCollisionModels")
    goal.addObject('MechanicalObject', name='GoalMO', showObject=True, drawMode="1", showObjectScale=3,
                             showColor=[0, 1, 0, 1], position=[0.0, -100.0, 100.0])
    '''


    #goal.addChild('visual')
    #goal.visual.addObject('MeshObjLoader', name='Loader', filename='mesh/ball.obj')
    #goal.visual.addObject('OglModel', name='BallOGL', src='@Loader')
    #goal.visual.addObject('RigidMapping', input='@../GoalMO', output='@BallOGL')

    goal = root.addChild("Goal")
    goal.addObject('MechanicalObject', name='GoalMO', showObject=True, drawMode="1", position=[pos[0], pos[1], pos[2]], template='Vec3d')
    goal.addObject('MeshObjLoader', name='loader', filename='mesh/ball.obj', scale3d=[3, 3, 3], translation=[pos[0], pos[1], pos[2]], triangulate=True)
    goal.addObject('OglModel', name='GoalOgl', src='@loader', color=[0, 1, 0, 1])
    #goal.addObject('RigidMapping', input='@GoalMO', output='@GoalOgl')
    
    return goal

def effectorTarget(parentNode, position=[0., 0., 200]):
    target = parentNode.addChild("Target")
    target.addObject("EulerImplicitSolver", firstOrder=True)
    target.addObject("CGLinearSolver")
    target.addObject("MechanicalObject", name="dofs", position=position, showObject=True, showObjectScale=3,
                     drawMode=2, showColor=[1., 1., 1., 1.])
    target.addObject("UncoupledConstraintCorrection")
    return target


#@SofaPrefab
class Trunk(SofaObject):
    """ This prefab is implementing a soft robot inspired by the elephant's trunk.
        The robot is entirely soft and actuated with 8 cables.
        The prefab is composed of:
        - a visual model
        - a collision model
        - a mechanical model for the deformable structure
        The prefab has the following parameters:
        - youngModulus
        - poissonRatio
        - totalMass
    """

    def __init__(self, parentNode, youngModulus=450, poissonRatio=0.45, totalMass=0.042, inverseMode=False):

        self.inverseMode = inverseMode
        self.node = parentNode.addChild('Trunk')

        self.node.addObject('MeshVTKLoader', name='loader', filename=path+'trunk.vtk')
        self.node.addObject('TetrahedronSetTopologyContainer', position="@loader.position", tetrahedra="@loader.tetrahedra", name='container')
        self.node.addObject('TetrahedronSetTopologyModifier')
        self.node.addObject('TetrahedronSetGeometryAlgorithms')

        self.node.addObject('MechanicalObject', name='dofs', template='Vec3d', showIndices='false',
                            showIndicesScale='4e-5')
        self.node.addObject('UniformMass', totalMass=totalMass)
        self.node.addObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large',
                            poissonRatio=poissonRatio,  youngModulus=youngModulus)

        self.__addCables()

    def __addCables(self):
        length1 = 10.
        length2 = 2.
        lengthTrunk = 195.

        pullPoint = [[0., length1, 0.], [-length1, 0., 0.], [0., -length1, 0.], [length1, 0., 0.]]
        direction = Vec3(0., length2-length1, lengthTrunk)
        direction.normalize()

        nbCables = 4

        self.cables = []
        for i in range(0, nbCables):
            theta = 1.57*i
            q = Quat(0., 0., sin(theta/2.), cos(theta/2.))

            position = [[0., 0., 0.]]*20
            for k in range(0, 20, 2):
                v = Vec3(direction[0], direction[1]*17.5*(k/2)+length1, direction[2]*17.5*(k/2)+21)
                position[k] = v.rotateFromQuat(q)
                v = Vec3(direction[0], direction[1]*17.5*(k/2)+length1, direction[2]*17.5*(k/2)+27)
                position[k+1] = v.rotateFromQuat(q)

            cableL = self.node.addChild('cableL'+str(i))
            cableL.addObject('MechanicalObject', name='meca', position=pullPoint[i]+[pos.toList() for pos in position])

            idx = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
            cableL.addObject('CableConstraint' if not self.inverseMode else 'CableActuator', template='Vec3d',
                             name="cable", hasPullPoint="0", indices=idx, maxPositiveDisp='70', maxDispVariation="1",
                             minForce=0)
            cableL.addObject('BarycentricMapping', name='mapping',  mapForces=False, mapMasses=False)
            self.cables.append(cableL)

        for i in range(0, nbCables):
            theta = 1.57*i
            q = Quat(0., 0., sin(theta/2.), cos(theta/2.))

            position = [[0., 0., 0.]]*10
            for k in range(0, 9, 2):
                v = Vec3(direction[0], direction[1]*17.5*(k/2)+length1, direction[2]*17.5*(k/2)+21)
                position[k] = v.rotateFromQuat(q)
                v = Vec3(direction[0], direction[1]*17.5*(k/2)+length1, direction[2]*17.5*(k/2)+27)
                position[k+1] = v.rotateFromQuat(q)

            cableS = self.node.addChild('cableS'+str(i))
            cableS.addObject('MechanicalObject', name='meca', position=pullPoint[i]+[pos.toList() for pos in position])

            idx = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
            cableS.addObject('CableConstraint' if not self.inverseMode else 'CableActuator', template='Vec3d',
                             name="cable", hasPullPoint="0", indices=idx, maxPositiveDisp='40', maxDispVariation="1",
                             minForce=0)
            cableS.addObject('BarycentricMapping', name='mapping',  mapForces='false', mapMasses='false')
            self.cables.append(cableS)

    def addVisualModel(self, color=[1., 1., 1., 1.]):
        trunkVisu = self.node.addChild('VisualModel')
        trunkVisu.addObject('MeshSTLLoader', name='trunkSTL', filename=path+"trunk.stl")
        trunkVisu.addObject('OglModel', name= 'trunkOgl', template='Vec3d', color=color)
        trunkVisu.addObject('BarycentricMapping')

    def addCollisionModel(self, selfCollision=False):
        trunkColli = self.node.addChild('CollisionModel')
        for i in range(2):
            part = trunkColli.addChild("Part"+str(i+1))
            part.addObject('MeshSTLLoader', name="loader", filename=path+"trunk_colli"+str(i+1)+".stl")
            part.addObject('MeshTopology', src="@loader")
            part.addObject('MechanicalObject')
            part.addObject('TTriangleModel', group=1 if not selfCollision else i)
            part.addObject('TLineModel', group=1 if not selfCollision else i)
            part.addObject('TPointModel', group=1 if not selfCollision else i)
            part.addObject('BarycentricMapping')

    def fixExtremity(self):
        self.node.addObject('BoxROI', name='boxROI', box=[[-20, -20, 0], [20, 20, 20]], drawBoxes=False)
        self.node.addObject('PartialFixedConstraint', fixedDirections="1 1 1", indices="@boxROI.indices")

    def addEffectors(self, target, position=[0., 0., 195.]):
        effectors = self.node.addChild("Effectors")
        effectors.addObject("MechanicalObject", position=position)
        effectors.addObject("BarycentricMapping", mapForces=False, mapMasses=False)


class TrunkVis(Sofa.Core.Controller):
    def __init__(self, viewer, root, env, *args, **kwargs):
        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.root = root
        self.env = env
        self.goal = self.root.Goal

        self.vedo_viewer = viewer
        self.camera = self.vedo_viewer.camera
        self.camera.SetPosition(500, 450, -500)
        
        self.vedo_trunk_mesh = Mesh()
        self.vedo_goal_mesh = Mesh()

    def onSimulationInitDoneEvent(self, _):
        # Init the Vedo Mesh
        trunk_positions = self.env.node.VisualModel.getObject('trunkOgl').position.value
        trunk_triangles = self.env.node.VisualModel.getObject('trunkOgl').triangles.value

        self.goal = self.root.Goal
        #print("----------------GOAL POS:", self.goal.loader.translation.value)
        goal_positions = self.goal.getObject('GoalOgl').position.value
        goal_quads = self.goal.getObject('GoalOgl').triangles.value

        self.vedo_trunk_mesh = Mesh(inputobj=[trunk_positions, trunk_triangles], c='blue5').wireframe(False).linewidth(1.5)
        self.vedo_goal_mesh = Mesh(inputobj=[goal_positions, goal_quads], c='green5').wireframe(False).linewidth(1.5)
        actors = [self.vedo_trunk_mesh, self.vedo_goal_mesh]

        # Init the Vedo Viewer
        self.vedo_viewer.add(actors)
        #self.vedo_viewer.show(actors, mode=0)

    def onAnimateEndEvent(self, _):
        # Update the Vedo Mesh (can be updated on place)
        trunk_positions = self.env.node.VisualModel.getObject('trunkOgl').position.value
        self.vedo_trunk_mesh.points(trunk_positions)
        
        goal_positions = self.goal.getObject('GoalOgl').position.value
        self.vedo_goal_mesh.points(goal_positions)

        # Update the Vedo Viewer
        self.vedo_viewer.render()




def createScene(rootNode, config={"source": [-600.0, -25, 100],
                                  "target": [30, -25, 100],
                                  "goalPos": [0, 0, 0]}, mode='simu_and_visu', viewer=None):

    # Chose the mode: visualization or computations (or both)
    visu, simu = False, False
    if 'visu' in mode:
        visu = True
    if 'simu' in mode:
        simu = True

    rootNode.addObject("RequiredPlugin", name="SoftRobots")
    rootNode.addObject("RequiredPlugin", name="SofaSparseSolver")
    rootNode.addObject("RequiredPlugin", name="SofaPreconditioner")
    rootNode.addObject("RequiredPlugin", name="SofaPython3")
    rootNode.addObject('RequiredPlugin', name='BeamAdapter')
    rootNode.addObject('RequiredPlugin', name='SofaOpenglVisual')
    rootNode.addObject('RequiredPlugin', name="SofaMiscCollision")
    rootNode.addObject("RequiredPlugin", name="SofaBoundaryCondition")
    rootNode.addObject("RequiredPlugin", name="SofaConstraint")
    rootNode.addObject("RequiredPlugin", name="SofaEngine")
    rootNode.addObject('RequiredPlugin', name='SofaImplicitOdeSolver')
    rootNode.addObject('RequiredPlugin', name='SofaLoader')
    rootNode.addObject('RequiredPlugin', name="SofaSimpleFem")

    if visu:
        source = config["source"]
        target = config["target"]
        rootNode.addObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels hideCollisionModels '
                                                       'hideMappings hideForceFields showWireframe')
        rootNode.addObject("LightManager")

        spotLoc = [2*source[0], 0, 0]
        rootNode.addObject("SpotLight", position=spotLoc, direction=[-np.sign(source[0]), 0.0, 0.0])
        rootNode.addObject("InteractiveCamera", name='camera', position=source, lookAt=target, zFar=500)
        rootNode.addObject('BackgroundSetting', color=[1, 1, 1, 1])
    if simu:
        rootNode.addObject('DefaultPipeline')
        rootNode.addObject('FreeMotionAnimationLoop')
        rootNode.addObject('GenericConstraintSolver', tolerance="1e-6", maxIterations="1000")
        rootNode.addObject('BruteForceDetection')
        rootNode.addObject('RuleBasedContactManager', responseParams="mu="+str(0.3), name='Response',
                           response='FrictionContactConstraint')
        rootNode.addObject('LocalMinDistance', alarmDistance=10, contactDistance=5, angleCone=0.01)

        rootNode.addObject(AnimationManagerController(rootNode))

        rootNode.gravity.value = [0., -9810., 0.]

    rootNode.dt.value = 0.01

    simulation = rootNode.addChild("Simulation")

    if simu:
        simulation.addObject('EulerImplicitSolver', name='odesolver', firstOrder="0", rayleighMass="0.1",
                             rayleighStiffness="0.1")
        simulation.addObject('EigenSimplicialLDLT',template='CompressedRowSparseMatrixd', name='linearSolver')
        simulation.addObject('GenericConstraintCorrection', solverName="@linearSolver")

    trunk = Trunk(simulation, inverseMode=False)
    rootNode.trunk = trunk

    trunk.fixExtremity()

    goal = add_goal_node(rootNode, config["goalPos"])

    rootNode.addObject(rewardShaper(name="Reward", rootNode=rootNode, goalPos=config['goalPos']))
    rootNode.addObject(goalSetter(name="GoalSetter", goalMO=goal.loader, goalPos=config['goalPos']))

    if visu:
        trunk.addVisualModel(color=[1., 1., 1., 0.8])

        t = rootNode.addObject(TrunkVis(viewer=viewer, root=rootNode, env=trunk, name="Trunk Visualization") )

    return rootNode
