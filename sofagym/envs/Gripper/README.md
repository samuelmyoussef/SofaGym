# Gripper
<center>
    <img src="../../../images/gripper-v0.png" width="500"/>


  <table>
    <tr>
      <td><b>Action Space</b></td>
      <td>Discrete(8)</td>
    </tr>
    <tr>
      <td><b>Observation Space</b></td>
      <td>Box([-1]*31, [1]*31, (31,), float32)</td>
    </tr>
    <tr>
      <td><b>Import</b></td>
      <td>gym.make("gripper-v0")</td>
    </tr>
  </table>
</center>


## Description
This environment represents a soft gripper employing two soft pneumatic actuators as the fingers. The objective is to grasp a cube and bring it to a certain height. The closer the cube is to the target, the greater the reward.


## Action Space
The action is discrete in the range `{0, 7}`. ....


## Observation Space
The observation is a ndarray with shape `(31,)` with the values between `-1` and `1` corresponding to ....


## Rewards
The reward is ....


## Starting State
The episode starts with the gripper and the cube in their initial positions ....


## Arguments
This environment currently has no additional arguments.

```python
import gym
import sofagym
from sofagym.envs import *

gym.make('gripper-v0')
```


## Episode End
The episode ends if any one of the following occurs:
- Termination: 
  1. ....
- Truncation (when using the time_limit wrapper): 
  1. The length of the episode reaches the limit specified using the `TimeLimit` wrapper.


## Version History
v0: Initial versions release