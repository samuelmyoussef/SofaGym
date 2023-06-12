# BubbleMotion


<center>
    <img src="../../../images/bubblemotion-v0.png" width="500"/>

  <table>
    <tr>
      <td><b>Action Space</b></td>
      <td>Box([-1]*9, [1]*9, (9,), float32)</td>
    </tr>
    <tr>
      <td><b>Observation Space</b></td>
      <td>Box([0]*15, [80]*15, (15,), float32)</td>
    </tr>
    <tr>
      <td><b>Import</b></td>
      <td>gym.make("bubblemotion-v0")</td>
    </tr>
  </table>
</center>


## Description
The aim of the `bubblemotion` environment is to move the sphere bubble to the specifed goal.


## Action Space


An action is a ndarray with shape`(9,)` which can take values between `-1` and `1`. ....


## Observation Space
The observation is a ndarray with shape `(15,)` with values between `0` and `80` corresponding to ....


## Rewards
The reward is the negative ratio of the current euclidean distance of the bubble from the goal and its previous distance.

$$
r = -{current\_distance \over previous\_distance} 
$$


## Starting State
The episode starts with the bubble and the goal initialized to random positions within the workspace.


## Arguments
This environment currently has no additional arguments.

```python
import gym
import sofagym
from sofagym.envs import *

gym.make('bubblemotion-v0')
```


## Episode End
The episode ends if any one of the following occurs:
- Termination: 
  1. The bubble reaches the goal
- Truncation (when using the time_limit wrapper): 
  1. The length of the episode reaches the limit specified using the `TimeLimit` wrapper.


## Version History
v0: Initial versions release