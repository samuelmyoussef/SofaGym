# Diamond
<center>
    <img src="../../../images/diamondrobot-v0.png" width="500"/>


  <table>
    <tr>
      <td><b>Action Space</b></td>
      <td>Discrete(8)</td>
    </tr>
    <tr>
      <td><b>Observation Space</b></td>
      <td>Box([-1]*5, [1]*5, (5,), float32)</td>
    </tr>
    <tr>
      <td><b>Import</b></td>
      <td>gym.make("diamondrobot-v0")</td>
    </tr>
  </table>
</center>


## Description
The diamond robot is a soft robot actuated by cables. The aim of this environment is to control the robot's tip to reach a specified target goal within its workspace.


## Action Space
The  diamond robot  is  controlled  by  four  cables  that can be contracted or extended by `0.05` unit.  There are therefore 8 possible actions. 

The action is discrete in the range `{0, 7}`. Actions 0 and 1 control the first cable by extending or contracting it by +0.05 or -0.05 displacement, respectively. Similarly, each two consecutive actions control the extension and contraction of one cable.

- 0: cable 1, extension
- 1: cable 1, contraction
- 2: cable 2, extension
- 3: cable 2, contraction
- 4: cable 3, extension
- 5: cable 3, contraction
- 6: cable 4, extension
- 7: cable 4, contraction


## Observation Space
The observation is a ndarray with shape `(5,)` with the values between `-1` and `1` corresponding to ....


## Rewards
The reward is the normalized value of the difference between the previous distance and the current distance of the tip from the goal. It has a value between 0 and 1 for each step.


## Starting State
The episode starts with the diamond robot in its initial position and the goal is initialized to a random position within the robot's workspace.


## Arguments
This environment currently has no additional arguments.

```python
import gym
import sofagym
from sofagym.envs import *

gym.make('diamondrobot-v0')
```


## Episode End
The episode ends if any one of the following occurs:
- Termination: 
  1. The diamond's tip reaches the goal.
- Truncation (when using the time_limit wrapper): 
  1. The length of the episode reaches the limit specified using the `TimeLimit` wrapper.


## Version History
v0: Initial versions release