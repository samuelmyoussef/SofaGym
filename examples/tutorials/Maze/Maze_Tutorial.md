# Maze in SofaGym Tutorial

This tutorial is a continuation to the [tripod tutorial](https://github.com/SofaDefrost/SoftRobots/tree/master/examples/tutorials/Tripod) from the SoftRobots plugin. The tripod tutorial covers building the scene for the tripod robot in SOFA and attaching a maze to it, then using a motion planning algorithm to solve the path the ball should take within the maze to reach its goal. Inverse control of the soft robot is then used to make the tripod robot solve the maze.

In this tutorial, we will describe how to use the [SofaGym](https://github.com/SofaDefrost/SofaGym/) plugin to create a [Gym](https://github.com/Farama-Foundation/Gymnasium) environment for the tripod robot and the maze to allow us to investigate solving the same control problem using Reinforcement Learning (RL) by training an RL agent of the robot in the SOFA simulation.

The two main parts of this tutorial focus on creating the gym environment for the tripod scene we previously created and want to train, and modifying the scene to add the necessary components to run the RL algorithms during the SOFA simulation.

Once completed, the knowledge acquired from this tutorial can be applied to other SOFA scenes.

Tutorial prequisites:

- you have installed [Sofa](https://www.sofa-framework.org/) and the [SofaGym](https://github.com/SofaDefrost/SofaGym/) plugin with all the necessary dependencies.

- you have basic knowledge of the [Python](https://www.python.org/) programming language. If this is not the case you can go to [Python Tutorials](https://docs.python.org/3/tutorial/index.html).

- you have basic knowledge of scene modelling with SOFA. If not, please complete the [FirstSteps](https://github.com/SofaDefrost/SoftRobots/tree/master/examples/tutorials/FirstSteps) tutorial first.

- you have completed the [tripod tutorial](https://github.com/SofaDefrost/SoftRobots/tree/master/examples/tutorials/Tripod).

<center>
<figure>
  <img src="../../../images/maze-v0.png" alt="" width="800px"/>
  <figcaption>Figure 1: Photo of the Tripod Robot with the Maze.</figcaption>
</figure>
</center>


# Step 1: Create the MazeEnv Class

The first step is to create a custom gym environment for the SOFA scene we want to simulate and train. For an in-depth documentaton on this, you can check the gymnasium [tutotrial](https://gymnasium.farama.org/tutorials/gymnasium_basics/environment_creation/#sphx-glr-tutorials-gymnasium-basics-environment-creation-py) for creating custom environments.

All gym environments are defined as classes that must inherit from the base `gym.Env` class and override the necessary methods. In SofaGym, our `AbstractEnv` class inherits from `gym.Env` to act as the base class for creating the SOFA gym environments. To create a new environment from a SOFA scene, you need to create a class for your environment that inherits from the `AbstractEnv` class. For this, we will create a new file `MazeEnv.py`.

```python
class MazeEnv(AbstractEnv):
    """Sub-class of AbstractEnv, dedicated to the tripod and maze scene.

    See the class AbstractEnv for arguments and methods.
    """
```

After creating the new environment class, default configuration for the SOFA scene should be defined as a dictionary. The minimum required data that must be defined can be found in [`AbstractEnv` documentation](https://github.com/SofaDefrost/SofaGym/blob/main/sofagym/AbstractEnv.py#L45-L76). Other config data can be added if needed, according to the scene.

```python
#Setting a default configuration
    path = os.path.dirname(os.path.abspath(__file__))
    metadata = {'render.modes': ['human', 'rgb_array']}
    DEFAULT_CONFIG = {"scene": "Maze",
                      "deterministic": True,
                      "source": [-82.0819, 186.518, 135.963],
                      "target": [-2.09447, 5.75347, -4.34572],
                      "goalList": [334, 317, 312, 301],
                      "goal_node": 270,
                      "start_node": 269,
                      "scale_factor": 5,
                      "timer_limit": 250,
                      "timeout": 50,
                      "display_size": (1600, 800),
                      "render": 1,
                      "save_data": False,
                      "save_image": False,
                      "save_path": path + "/Results" + "/Maze",
                      "planning": True,
                      "discrete": True,
                      "seed": 0,
                      "start_from_history": None,
                      "python_version": "python3.8",
                      "zFar": 1000,
                      "dt": 0.01,
                      "time_before_start": 20,
                      }
```

In the class `init`, we initialize any necessary varialbes and assign their values based on the previously defined config.

We must also define the type of actions and observations the gym environment will use by defining the `action_space` and `observation_space`. For the Maze scene, we define 6 possible actions as `Discrete(6)` since we have 3 servo motors and we have 2 actions for each motor to control whether to increase or decrease the servo horn angle by a specified step angle. The observation space is defined as `Box()` that consists of 9 continuous values representing the xyz positions of the ball, the maze, and the goal point.

```python
    def __init__(self, config = None):
        super().__init__(config)
        nb_actions = 6
        self.action_space = spaces.Discrete(nb_actions)
        self.nb_actions = str(nb_actions)

        dim_state = 9
        low_coordinates = np.array([-1]*dim_state)
        high_coordinates = np.array([1]*dim_state)
        self.observation_space = spaces.Box(low_coordinates, high_coordinates, dtype='float32')
```

The second part of is to override the step and reset methods from the `AbstractEnv` class. For the step method, no additions are required. For the reset method, we need to restart the scene using `start_scene` and return the first observation. It also updates the new goal position as it is randomly chosen from a list of defined points. The list of goal points is set in `DEFAULT_CONFIG["goalList"]` to the 4 corners of the maze.

```python
    def step(self, action):
        return super().step(action)

    def reset(self):
        """Reset simulation.

        Note:
        ----
            We launch a client to create the scene. The scene of the program is
            client_<scene>Env.py.

        """
        super().reset()

        self.config.update({'goalPos': self.goal})

        obs = start_scene(self.config, self.nb_actions)
        
        return np.array(obs['observation'])
```


# Step 2: Create the Environment's Toolbox

The next step is to implement the methods and functions necessary for the RL algorithms to work. Three essential parts need to be implemented for this, defining and applying the actions, defining and calculating the reward, and getting the new observation from the simulation and updating it. One additional component is a `GoalSetter` to define and update a target goal. The `GoalSetter` could be optional in the case of some environments but it is needed for this maze environment.

First, we need to creat a new file `MazeToolbox`. For each of the parts that need to be implemented, some components are required to be defined to make the SOFA scene compatible with SofaGym.
- Actions:
  - startCmd function
- Reward:
  - getReward function
  - rewardShaper "Reward" class
- Observation:
  - getState function
  - getPos function
  - setPos function
- Goal:
  - goalSetter "GoalSetter" class


## Actions
The possible actions that could be applied by the agent must be defined. The Maze environment has 6 actions as discussed in the previous step, increase or decrease the angle of each of the servo motors. This is done by applying a small increment or decrement of 0.1 rad (about 5 degrees) to the motor's horn angle value.

First, the `action_to_command` function is used to convert the action value chosen by the RL algorithm, whcih is between 0 and 5, to the number of the actuator and the angle change value of either 0.1 or -0.1.

Then, the `displace` function is defined to apply the returned command to the simulation by changing the angle input value of the chosen actuator to the specified value. The actuator's horn angle is restricted to 90 degrees in both directions from its center neutral position.

```python
def action_to_command(action):
    """Link between Gym action (int) and SOFA command (displacement of cables).

    Parameters:
    ----------
        action: int
            The number of the action (Gym).

    Returns:
    -------
        The command (number of the cabl and its displacement).
    """
    if action == 0:
        num_actuator, displacement = 0, 0.1
    elif action == 1:
        num_actuator, displacement = 1, 0.1
    elif action == 2:
        num_actuator, displacement = 2, 0.1
    elif action == 3:
        num_actuator, displacement = 0, -0.1
    elif action == 4:
        num_actuator, displacement = 1, -0.1
    elif action == 5:
        num_actuator, displacement = 2, -0.1
    else:
        raise NotImplementedError("Action is not in range 0, 5")

    return num_actuator, displacement

def displace(actuator, displacement):
    """Change the value of the angle.

    Parameters:
    ----------
        acuator:
            The motor we consider.
        displacement: int
            The increment for the angle.

    Returns:
    -------
        None.
    """
    new_value = actuator.angleIn.value + displacement
    if new_value <= 1.5 and new_value >= -1.5:
        actuator.angleIn.value = new_value
```

Next, the required part is to define the `startCmd` function, which is used by SofaGym as a link between Gym and SOFA to execute the actions as SOFA commands in the simulation.

A helper function for the environment can be defined first. The `startCmd_Maze` function is used to call the SOFA `AnimationManager` to execute the necessary command in the simulation based on the chosen action and run the animation. After defining this function, it is simple to define the `startCmd` function to get the needed command and apply it to the simulation.

```python
def startCmd_Maze(rootNode, actuator, displacement, duration):
    """Initialize the command.

    Parameters:
    ----------
        rootNode: <Sofa.Core>
            The root.
        acuator:
            The motor we consider.
        displacement: int
            The increment for the angle.
        duration: float
            Duration of the animation.

    Returns:
    -------
        None.
    """

    # Definition of the elements of the animation
    def executeAnimation(actuator, displacement, factor):
        displace(actuator, displacement)

    # Add animation in the scene
    rootNode.AnimationManager.addAnimation(
        Animation(onUpdate=executeAnimation, params={"actuator": actuator, "displacement": displacement},
                  duration=duration, mode="once"))

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
    num_actuator, displacement = action_to_command(action)
    actuators = [root.Modelling.Tripod.ActuatedArm0,
                 root.Modelling.Tripod.ActuatedArm1,
                 root.Modelling.Tripod.ActuatedArm2]
    actuator = actuators[num_actuator]

    startCmd_Maze(root, actuator, displacement, duration)
```

## Reward
For the reward, we define a `rewardShaper` class to inherit from `Sofa.Core.Controller` to update the reward value at each simulation step. In the initialization, we can define some parameters to be used for the reward calculation based on the scene configs, such as the goal point (`goal_node`), the ball position (`ball_mo`), and the maze's path (`path_mesh` and `path_mo`).

Depending on the scene, some helper methods, classes, or functions could be defined to get or calculate different values. For the Maze scene, we create a new file `MazeTools.py` to define them. We first define a `Graph` class to store the graph tree of the shortest path the ball should follow to the goal. We also define an implementation of the `dijkstra` algorithm to calculate that shortest path.

```python
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
```

Next, we can use the shortest path we calculate to determine the reward. First, an `update` method is defined for the `rewardShaper` to calculate the shortest path and store it in the graph at the beginning of each episode.

Then, in the `getReward` method, we get the ball position and check how close it is to following the path. The reward is then calculated as a normalized ratio of the distance between the ball and the closest point in the path.

```python
class rewardShaper(Sofa.Core.Controller):
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
            self.rootNode = kwargs["rootNode"]
        self.goal_node = None
        if kwargs["goal_node"]:
            self.goal_node = kwargs["goal_node"]
        self.path_mesh = None
        if kwargs["path_mesh"]:
            self.path_mesh = kwargs["path_mesh"]
        self.path_mo = None
        if kwargs["path_mo"]:
            self.path_mo = kwargs["path_mo"]
        self.ball_mo = None
        if kwargs["ball_mo"]:
            self.ball_mo = kwargs["ball_mo"]

        self.start_node = 115
        self.prev_ratio = 0.0

    def getReward(self):
        """Compute the reward.

        Parameters:
        ----------
            None.

        Returns:
        -------
            The reward and the cost.

        """
        pos = self.ball_mo.position.value[0]
        dist_to_path = [{'id': k, 'dist': np.linalg.norm(pos - path_point)}
                        for k, path_point in enumerate(self.path_pos)]
        sorted_dist = sorted(dist_to_path, key=lambda item: item['dist'])
        if len(sorted_dist) < 2:
            return 0.0
        closest_points = [sorted_dist[0]["id"], sorted_dist[1]["id"]]

        new_ratio = max(max(self.path_length[closest_points[0]],
                            self.path_length[closest_points[1]]), 0)/self.path_length[-1]
        if new_ratio > self.prev_ratio:
            self.prev_ratio = new_ratio
            return 1.0, None
        else:
            return new_ratio, None

    def update(self):
        """Update function.

        This function is used as an initialization function.

        Parameters:
        ----------
            None.

        Arguments:
        ---------
            None.

        """
        edges = []
        with self.path_mesh.edges.writeable() as Topoedges:
            for edge in Topoedges:
                edges += [(edge[0], edge[1], np.linalg.norm(np.array(self.path_mesh.position.value[edge[0]]) -
                                                            np.array(self.path_mesh.position.value[edge[1]])))]

        self.path_graph = Graph()
        for edge in edges:
            self.path_graph.add_edge(*edge)

        path1, path_length1 = dijkstra(self.path_graph, 30, self.goal_node)
        path2, path_length2 = dijkstra(self.path_graph, 31, self.goal_node)
        if len(path1) > len(path2):
            self.path, self.path_length = path1, path_length1
        else:
            self.path, self.path_length = path2, path_length2
        self.path_pos = []
        for point in self.path:
            self.path_pos += [self.path_mo.position.value[point][:3]]
```

The `getReward` function must be defined to be used by SofaGym to calculate the reward value and done state and return them by Gym at each step. We simply use the `rewardShaper` class we just defined to get the reward and check if the episode is done based on the termination condition. For the Maze, the episode ends if the ball falls off the maze or if it successfully reaches the goal point, so we check the distance between the ball and goal point to determine this.

```python
def getReward(rootNode):
    """Compute the reward using Reward.getReward().

    Parameters:
    ----------
        rootNode: <Sofa.Core>
            The scene.

    Returns:
    -------
        done, reward

    """
    done = False

    goal = rootNode.Goal.GoalMO.position.value[0]
    ball_pos = rootNode.Simulation.Sphere.sphere_mo.position.value[0]
    dist = np.linalg.norm(ball_pos - goal)

    reward, cost = rootNode.Reward.getReward()

    if dist < 5 or ball_pos[1] < -100:
        done = True

    return done, reward
```

## Observations

After applying the action to the simulation scene and calculating the retured reward, the new state of the environment must also be returned. To do this, it is required to define a `getState` function to get and calculate the new state and return it. As discussed in step 1, the Maze environment's state at each step consists of 9 values of the xyz positions of the ball, the maze, and the goal. The `_getGoalPos` position is used to get the position of the goal. The positions of the ball and the maze are also updated from the scene and all the positions can be concatenated and returned at each step.

```python
def _getGoalPos(rootNode):
    """Get XYZ position of the goal.

    Parameters:
    ----------
        rootNode: <Sofa.Core>
            The scene.

    Returns:
    -------
        The position of the goal.
    """
    return rootNode.Goal.GoalMO.position[0]


def getState(rootNode):
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
    cs = 3

    goalPos = _getGoalPos(rootNode).tolist()
    maze = rootNode.Modelling.Tripod.RigidifiedStructure.FreeCenter.Maze.maze_mesh_mo.position.value[0]
    maze = [round(float(k), cs) for k in maze]

    spheres = rootNode.Simulation.Sphere.sphere_mo.position.value[0]
    spheres = [round(float(k), cs) for k in spheres]

    state = spheres + maze + goalPos

    return state
```

The second required part is to define two functions: `getPos` and `setPos` to retrieve the positions of all the components that will be visualized in the scene at the beginning of each step and update them at the end of the step with the new state. These two functions are mainly needed for the rendering part.

```python
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
    maze = root.Modelling.Tripod.RigidifiedStructure.FreeCenter.Maze.maze_mesh_mo.position.value.tolist()
    spheres = root.Simulation.Sphere.sphere_mo.position.value.tolist()

    rigid = root.Modelling.Tripod.RigidifiedStructure.RigidParts.dofs.position.value.tolist()
    deformable = root.Modelling.Tripod.RigidifiedStructure.DeformableParts.dofs.position.value.tolist()
    elastic = root.Modelling.Tripod.ElasticBody.MechanicalModel.dofs.position.value.tolist()
    freecenter = root.Modelling.Tripod.RigidifiedStructure.FreeCenter.dofs.position.value.tolist()

    arm1 = root.Modelling.Tripod.ActuatedArm0.ServoMotor.Articulation.dofs.position.value.tolist()
    arm2 = root.Modelling.Tripod.ActuatedArm1.ServoMotor.Articulation.dofs.position.value.tolist()
    arm3 = root.Modelling.Tripod.ActuatedArm2.ServoMotor.Articulation.dofs.position.value.tolist()

    servo1 = root.Modelling.Tripod.ActuatedArm0.ServoMotor.ServoBody.dofs.position.value.tolist()
    servo2 = root.Modelling.Tripod.ActuatedArm1.ServoMotor.ServoBody.dofs.position.value.tolist()
    servo3 = root.Modelling.Tripod.ActuatedArm2.ServoMotor.ServoBody.dofs.position.value.tolist()

    goal = root.Goal.GoalMO.position.value.tolist()

    return [maze, spheres, rigid, deformable, elastic, freecenter, arm1, arm2, arm3, servo1, servo2, servo3, goal]

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
    [maze, spheres, rigid, deformable, elastic, freecenter, arm1, arm2, arm3, servo1, servo2, servo3, goal] = pos

    root.Modelling.Tripod.RigidifiedStructure.FreeCenter.Maze.maze_mesh_mo.position.value = np.array(maze)
    root.Simulation.Sphere.sphere_mo.position.value = np.array(spheres)

    root.Modelling.Tripod.RigidifiedStructure.RigidParts.dofs.position.value = np.array(rigid)
    root.Modelling.Tripod.RigidifiedStructure.DeformableParts.dofs.position.value = np.array(deformable)
    root.Modelling.Tripod.ElasticBody.MechanicalModel.dofs.position.value = np.array(elastic)
    root.Modelling.Tripod.RigidifiedStructure.FreeCenter.dofs.position.value = np.array(freecenter)

    root.Modelling.Tripod.ActuatedArm0.ServoMotor.Articulation.dofs.position.value = np.array(arm1)
    root.Modelling.Tripod.ActuatedArm1.ServoMotor.Articulation.dofs.position.value = np.array(arm2)
    root.Modelling.Tripod.ActuatedArm2.ServoMotor.Articulation.dofs.position.value = np.array(arm3)

    root.Modelling.Tripod.ActuatedArm0.ServoMotor.ServoBody.dofs.position.value = np.array(servo1)
    root.Modelling.Tripod.ActuatedArm1.ServoMotor.ServoBody.dofs.position.value = np.array(servo2)
    root.Modelling.Tripod.ActuatedArm2.ServoMotor.ServoBody.dofs.position.value = np.array(servo3)

    root.Goal.GoalMO.position.value = np.array(goal)

```

## Goal

This step is only feasible for environments or scenes where a goal is defined, such as the target position that the ball needs to reach within the maze in this case. The `GoalSetter` class is used to initialize the goal in the scene and randomly it at the start of each episode.

Similar to the `rewardShaper`, we first initialize some needed components such as the `goal` object and the goal position `goalPos`. The `update` method randomizes the position of the goal at the beginning of each new episode based on a new goal id chosen from a list of 4 random points representing the corners of the maze.

```python
class goalSetter(Sofa.Core.Controller):
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

    def update(self):
        """Set the position of the goal.

        This function is used as an initialization function.

        Parameters:
        ----------
            None.

        Arguments:
        ---------
            None.

        """
        new_position = self.rootNode.Modelling.Tripod.RigidifiedStructure.FreeCenter.Maze.Path.dofs.position.value[self.goalPos][:3]
        with self.goal.GoalMO.position.writeable() as position:
            position[0] = new_position
        with self.goal.mapping.initialPoints.writeable() as position:
            position[0] = new_position
            position[0][1] = 5

    def set_mo_pos(self, goal):
        """Modify the goal.

        Not used here.
        """
        pass
```


# Step 3: Modify the Scene

The next step is to modify the scene to include the components we defined in the toolbox in the previous step. We need to modify the `MazeScene` file.

In this step, we simply add objects of the three classes we defined to the root node of the scene with the required parameters: `rewardShaper`, and `GoalSetter`.

```python
    # SofaGym Env Components
    rootNode.addObject(rewardShaper(name="Reward", rootNode=rootNode, goal_node=config['goalPos'],
                                    path_mesh=p_mesh, path_mo=p_mo, ball_mo=ball_mo))
    rootNode.addObject(goalSetter(name="GoalSetter", rootNode=rootNode, goal=goal, goalPos=config['goalPos']))
```

Finally, the goal is added to the scene as a `Mechanical Object` attached to the maze.

```python
def add_goal_node(root):
    goal = root.addChild("Goal")
    goal.addObject('VisualStyle', displayFlags="showCollisionModels")
    goal_mo = goal.addObject('MechanicalObject', name='GoalMO', showObject=True, drawMode="1", showObjectScale=3,
                             showColor=[0, 1, 0, 1], position=[0.0, 0.0, 0.0])
    goal.addObject("RigidMapping", name='mapping', input=root.Modelling.Tripod.RigidifiedStructure.FreeCenter.dofs.getLinkPath(), output=goal_mo.getLinkPath())

    return goal
```

# Step 4: Register the New Environment in Gym

The final step is to register the new environment so that Gym can locate it and run it. To do this, we must modify the [`__init__.py` file](https://github.com/SofaDefrost/SofaGym/blob/main/sofagym/envs/__init__.py). The `id` is the name that will be used to run the environment in gym ```gym.make('maze-v0')```. The `entry point` is the environment class we created in step 1 `MazeEnv`.

```python
from sofagym.envs.MazeEnv.MazeEnv import *
register(
    id='maze-v0',
    entry_point='sofagym.envs:MazeEnv',
)
```

To use the new environment with the test and training scripts in SofaGym, the environment id must be added to the envs dicts in the respective files.

In [`test_env.py`](https://github.com/SofaDefrost/SofaGym/blob/main/test_env.py):
```python
name = {
        1:'bubblemotion-v0',
        2:'cartstem-v0',
        3:'cartstemcontact-v0',
        4:'catchtheobject-v0',
        5:'concentrictuberobot-v0',
        6:'diamondrobot-v0',
        7:'gripper-v0',
        8:'maze-v0',                    #Maze env
        9:'multigaitrobot-v0',
        10:'simple_maze-v0',
        11:'stempendulum-v0',
        12:'trunk-v0',
        13:'trunkcup-v0',
        }
```

In [`rl.py`](https://github.com/SofaDefrost/SofaGym/blob/main/rl.py):
```python
envs = {
        1: 'bubblemotion-v0',
        2: 'cartstem-v0',
        3: 'cartstemcontact-v0',
        4: 'catchtheobject-v0',
        5: 'concentrictuberobot-v0',
        6: 'diamondrobot-v0',
        7: 'gripper-v0',
        8: 'maze-v0',                   #Maze env
        9: 'multigaitrobot-v0',
        10: 'simple_maze-v0',
        11: 'stempendulum-v0',
        12: 'trunk-v0',
        13: 'trunkcup-v0',
        }
```