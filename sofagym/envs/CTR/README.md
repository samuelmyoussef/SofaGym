# ConcentricTubeRobot (CTR)
<center>
    <img src="../../../images/concentrictuberobot-v0.png" width="500"/>


  <table>
    <tr>
      <td><b>Action Space</b></td>
      <td>Discrete(12)</td>
    </tr>
    <tr>
      <td><b>Observation Space</b></td>
      <td>Box([-1], [1], (1,), float32)</td>
    </tr>
    <tr>
      <td><b>Import</b></td>
      <td>gym.make("concentrictuberobot-v0")</td>
    </tr>
  </table>
</center>


## Description
This environment is a simple representation of a concentric tube tool such as a catheter that navigates inside the body through blood vessels to reach a specified target goal.


## Action Space
The concentric tube robot (CTR) consists of 3 tubes. The action is discrete in the range `{0, 11}`. The actions control the scale of transaltion and rotation of the 3 tubes. Actions from 0 to 3, 4 to 7, and 8 to 11 control tubes 1, 2, and 3 respectively.


## Observation Space
The observation is a ndarray with shape `(1,)` with the values between `-1` and `1` corresponding to ....


## Rewards
The reward is the normalized value of the difference between the previous distance and the current distance of the CTR's tip from the goal. It has a value between 0 and 1 for each step.


## Starting State
The episode starts with the tube in its initial position and the goal is initialized to a random position within the workspace.


## Arguments
This environment currently has no additional arguments.

```python
import gym
import sofagym
from sofagym.envs import *

gym.make('concentrictuberobot-v0')
```


## Episode End
The episode ends if any one of the following occurs:
- Termination: 
  1. The tube reaches the goal.
- Truncation (when using the time_limit wrapper): 
  1. The length of the episode reaches the limit specified using the `TimeLimit` wrapper.


## Version History
v0: Initial versions release