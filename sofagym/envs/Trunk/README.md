# Trunk


<center>
    <img src="../../../images/trunk-v0.png" width="500"/>

  <table>
    <tr>
      <td><b>Action Space</b></td>
      <td>Discrete(16)</td>
    </tr>
    <tr>
      <td><b>Observation Space</b></td>
      <td>Box([-1]*66, [1]*66, (66,), float32)</td>
    </tr>
    <tr>
      <td><b>Import</b></td>
      <td>gym.make("trunk-v0")</td>
    </tr>
  </table>
</center>


## Description
The elephant trunk manipulator is an articulated tendon-driven soft manipulator that is actuated using cables. The aim of the Trunk environment is to bring the trunk’s tip to a certain position within its workspace.


## Action Space
The  trunk  is  controlled  by  eight  cables  that can be contracted or extended by one unit.  There are therefore `16` possible actions. The action space presented here is discrete but could easily be ex-tended to become continuous.

The action shape is `(1,)` in the range `{0, 15}`. Actions from 0 to 7 extend the appropriate cable by `+1` displacement, while actions from 8 to 15 contract it by -1 displacement unit.

- 0: cable 0, extension
- 1: cable 1, extension
- 2: cable 2, extension
- 3: cable 3, extension
- 4: cable 4, extension
- 5: cable 5, extension
- 6: cable 6, extension
- 7: cable 7, extension
- 8: cable 8, contraction
- 9: cable 9, contraction
- 10: cable 10, contraction
- 11: cable 11, contraction
- 12: cable 12, contraction
- 13: cable 13, contraction
- 14: cable 14, contraction
- 15: cable 15, contraction


## Observation Space
The observation is a ndarray with shape `(66,)` with the values between `-1` and `1` corresponding to ....


## Rewards
The reward is the normalized value of the difference between the previous and current euclidean distance of the tip from the goal. It has a value between `0` and `1` for each step.


## Starting State
The episode starts with the trunk in its initial position and the goal is initialized to a random position within the trunk's workspace.


## Arguments
This environment currently has no additional arguments.

```python
import gym
import sofagym
from sofagym.envs import *

gym.make('trunk-v0')
```


## Episode End
The episode ends if any one of the following occurs:
- Termination: 
  1. The trunk's tip reaches the goal.
- Truncation (when using the time_limit wrapper): 
  1. The length of the episode reaches the limit specified using the `TimeLimit` wrapper.


## Version History
v0: Initial versions release