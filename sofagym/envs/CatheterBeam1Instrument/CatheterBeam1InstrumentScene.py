import pathlib
import sys
from os.path import abspath, dirname

sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute())+"/../")
sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute()))

import numpy as np
from CatheterBeam1InstrumentToolbox import GoalSetter, RewardShaper, SimRestore
from splib3.animation import AnimationManagerController

from sofagym.controllers import RandomActions, ActionsSequence, ReloadSim

path = dirname(abspath(__file__)) + '/mesh/'


def add_goal_node(root):
    goal = root.addChild("Goal")
    goal.addObject('VisualStyle', displayFlags="showCollisionModels")
    goal_mo = goal.addObject('MechanicalObject', name='GoalMO', showObject=True, drawMode="1", showObjectScale=2.0,
                             showColor=[0, 1, 0, 0.5], position=[0.0, 0.0, 0.0])
    return goal


dim_state = 8
DEFAULT_CONFIG = {"scene": "CatheterBeam1Instrument",
                  "deterministic": True,
                  "source": [-1169.51, 298.574, 257.631],
                  "target": [0, 0, 0],
                  "start_node": None,
                  "scale_factor": 10,
                  "dt": 0.01,
                  "timer_limit": 80,
                  "timeout": 50,
                  "display_size": (1600, 800),
                  "render": 0,
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
                  "use_server": False,
                  "goalPos": None
                  }

actions_squence = [0, 0, 0, 0, 0, 0, 0, 0, 0 , 1]

def createScene(root,
                config=DEFAULT_CONFIG,
                mode='simu_and_visu'):
    
    # SETUP
    ## Choose the mode: visualization or computations (or both)
    visu, simu = False, False
    if 'visu' in mode:
        visu = True
    if 'simu' in mode:
        simu = True

    ## Root Parameters
    root.name = "root"
    root.gravity=[0.0, 0.0, 0.0]
    root.dt = config['dt']

    plugins_list = ["Sofa.Component.AnimationLoop",
                    "Sofa.Component.IO.Mesh",
                    "Sofa.Component.Mapping.Linear",
                    "Sofa.Component.Mapping.NonLinear",
                    "Sofa.Component.LinearSolver.Direct",
                    "Sofa.Component.LinearSolver.Iterative",
                    "Sofa.Component.ODESolver.Backward",
                    "Sofa.Component.Engine.Generate",
                    "Sofa.Component.Mass",
                    "Sofa.Component.MechanicalLoad",
                    "Sofa.Component.SolidMechanics.Spring",
                    "Sofa.Component.Constraint.Projective",
                    "Sofa.Component.Constraint.Lagrangian.Correction",
                    "Sofa.Component.Constraint.Lagrangian.Model",
                    "Sofa.Component.Constraint.Lagrangian.Solver",
                    "Sofa.Component.StateContainer",
                    "Sofa.Component.Topology.Container.Constant",
                    "Sofa.Component.Topology.Container.Dynamic",
                    "Sofa.Component.Topology.Container.Grid",
                    "Sofa.Component.Topology.Mapping",
                    "Sofa.Component.Collision.Detection.Algorithm",
                    "Sofa.Component.Collision.Detection.Intersection",
                    "Sofa.Component.Collision.Response.Contact",
                    "Sofa.Component.Collision.Geometry",
                    "Sofa.Component.Visual",
                    "Sofa.GL.Component.Rendering3D",
                    "Sofa.GL.Component.Shader",
                    "BeamAdapter"]
    
    plugins = root.addChild('Plugins')
    for name in plugins_list:
        plugins.addObject('RequiredPlugin', name=name, printLog=False)

    root.addObject('VisualStyle', displayFlags='showVisualModels showBehaviorModels hideMappings hideForceFields')
    root.addObject('DefaultVisualManagerLoop')

    root.addObject('FreeMotionAnimationLoop')
    root.addObject('LCPConstraintSolver', mu=0.1, tolerance=1e-10, maxIt=1000, build_lcp=False)

    #root.addObject('DefaultPipeline', depth=6, verbose=True, draw=False)
    root.addObject('BruteForceBroadPhase')
    root.addObject('BVHNarrowPhase')
    root.addObject('LocalMinDistance', alarmDistance=2, contactDistance=1, angleCone=0.8, coneFactor=0.8)
    root.addObject('DefaultContactManager', name='Response', response='FrictionContactConstraint')

    # SCENE
    ## Guide
    guide = root.addChild('topoLines_guide')
    guide.addObject('WireRestShape', template='Rigid3d', printLog=False, name='GuideRestShape', length=1000.0, straightLength=980.0, spireDiameter=25, spireHeight=0.0,
                    densityOfBeams=[30, 5], numEdges=200, numEdgesCollis=[50, 10], youngModulus=10000, youngModulusExtremity=10000)
    guide.addObject('EdgeSetTopologyContainer', name='meshLinesGuide')
    guide.addObject('EdgeSetTopologyModifier', name='Modifier')
    guide.addObject('EdgeSetGeometryAlgorithms', name='GeomAlgo', template='Rigid3d')
    guide.addObject('MechanicalObject', template='Rigid3d', name='dofTopo2')

    ## Combined Instrument
    instrument = root.addChild('InstrumentCombined')
    instrument.addObject('EulerImplicitSolver', rayleighStiffness=0.2, rayleighMass=0.1, printLog=False)
    instrument.addObject('BTDLinearSolver', subpartSolve=False, verification=False, verbose=False)
    instrument.addObject('RegularGridTopology', name='meshLinesCombined', nx=60, ny=1, nz=1, xmin=0.0, xmax=1.0, ymin=0, ymax=0, zmin=1, zmax=1)
    instrument.addObject('MechanicalObject', template='Rigid3d', name='DOFs', showIndices=False, ry=-90)
    
    instrument.addObject('WireBeamInterpolation', name='InterpolGuide', WireRestShape='@../topoLines_guide/GuideRestShape', radius=0.9, printLog=False)
    instrument.addObject('AdaptiveBeamForceFieldAndMass', name='GuideForceField', interpolation='@InterpolGuide', massDensity=0.00000155)

    instrument.addObject('InterventionalRadiologyController', template='Rigid3d', name='m_ircontroller', printLog=False, xtip=[0, 0, 0], step=3, rotationInstrument=[0, 0, 0],
                         controlledInstrument=0, startingPos=[0, 0, 0, 0, -0.7071068, 0, 0.7071068], speed=0, instruments='InterpolGuide')
    
    instrument.addObject('LinearSolverConstraintCorrection', printLog=False, wire_optimization=True)
    instrument.addObject('FixedConstraint', name='FixedConstraint', indices=0)
    instrument.addObject('RestShapeSpringsForceField', points='@m_ircontroller.indexFirstNode', stiffness=1e8, angularStiffness=1e8)
    
    collis = instrument.addChild('Collis', activated=True)
    collis.addObject('EdgeSetTopologyContainer', name='collisEdgeSet')
    collis.addObject('EdgeSetTopologyModifier', name='colliseEdgeModifier')
    collis.addObject('MechanicalObject', name='CollisionDOFs')
    collis.addObject('MultiAdaptiveBeamMapping', name='collisMap', controller='../m_ircontroller', useCurvAbs=True, printLog=False)
    collis.addObject('LineCollisionModel', proximity=0.0, group=1)
    collis.addObject('PointCollisionModel', proximity=0.0, group=1)
    
    guide_visu = instrument.addChild('VisuGuide', activated=True)
    guide_visu.addObject('MechanicalObject', name='Quads')
    guide_visu.addObject('QuadSetTopologyContainer', name='ContainerGuide')
    guide_visu.addObject('QuadSetTopologyModifier', name='Modifier')
    guide_visu.addObject('QuadSetGeometryAlgorithms', name='GeomAlgo', template='Vec3d')
    guide_visu.addObject('Edge2QuadTopologicalMapping', nbPointsOnEachCircle=10, radius=1, input='@../../topoLines_guide/meshLinesGuide', output='@ContainerGuide', flipNormals=True, listening=True)
    guide_visu.addObject('AdaptiveBeamMapping', name='visuMapGuide', useCurvAbs=True, printLog=False, interpolation='@../InterpolGuide', input='@../DOFs', output='@Quads', isMechanical=False)
			
    guide_visuOgl = guide_visu.addChild('VisuOgl')
    guide_visuOgl.addObject('OglModel', name='Visual', color=[0.2, 0.2, 0.8], material='texture Ambient 1 0.2 0.2 0.2 0.0 Diffuse 1 1.0 1.0 1.0 1.0 Specular 1 1.0 1.0 1.0 1.0 Emissive 0 0.15 0.05 0.05 0.0 Shininess 1 20', quads='@../ContainerGuide.quads')
    guide_visuOgl.addObject('IdentityMapping', input='@../Quads', output='@Visual')

    ## Collision
    collision = root.addChild('CollisionModel')
    collision.addObject('MeshOBJLoader', name='meshLoader', filename=path+'phantom.obj', triangulate=True, flipNormals=True)
    #collision.addObject('MeshSTLLoader', name='meshLoader', filename=path+'carotids.stl', triangulate=True, flipNormals=False, rotation=[10.0, 0.0, -90.0])
    collision.addObject('MeshTopology', position='@meshLoader.position', triangles='@meshLoader.triangles')
    collision.addObject('MechanicalObject', name='DOFs1', position=[0, 0, 400], scale=3, ry=90)
    collision.addObject('TriangleCollisionModel', simulated=False, moving=False)
    collision.addObject('LineCollisionModel', simulated=False, moving=False)
    collision.addObject('PointCollisionModel', simulated=False, moving=False)
    collision.addObject('OglModel', name='Visual', src='@meshLoader', color=[1, 0, 0, 0.1], scale=3, ry=90)

    # Goal
    goal = add_goal_node(root)

    # SofaGym Env Toolbox
    root.addObject(RewardShaper(name="Reward", rootNode=root, goalPos=config['goalPos']))
    root.addObject(GoalSetter(name="GoalSetter", rootNode=root, goal=goal, goalPos=config['goalPos']))

    if visu:
        source = config["source"]
        target = config["target"]
        root.addObject("LightManager")
        spotloc = [0, source[1]+config["zFar"], 0]
        root.addObject("SpotLight", position=spotloc, direction=[0, -np.sign(source[1]), 0])
        root.addObject("InteractiveCamera", name="camera", position=source, orientation=[0.472056, -0.599521, -0.501909, 0.407217], lookAt=target, zFar=5000)

    root.addObject(AnimationManagerController(root, name="AnimationManager"))

    #root.addObject(RandomActions(name="RandomActions", rootNode=root, env_id="catheter_beam_1_instrument-v0"))
    #root.addObject(ActionsSequence(name="ActionsSequence", rootNode=root, env_id="catheter_beam_1_instrument-v0", actions_sequence=actions_squence))

    root.addObject(SimRestore(name="SimRestore", rootNode=root))
    #root.addObject(ReloadSim(name="ReloadSim", rootNode=root))

    return root
