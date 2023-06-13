# StemPendulum
<center>
    <img src="../../../images/stempendulum-v0.png" width="500"/>


  <table>
    <tr>
      <td><b>Action Space</b></td>
      <td>Box([-1], [1], (1,), float32)</td>
    </tr>
    <tr>
      <td><b>Observation Space</b></td>
      <td>Box([-2]*5, [2]*5, (5,), float32)</td>
    </tr>
    <tr>
      <td><b>Import</b></td>
      <td>gym.make("stempendulum-v0")</td>
    </tr>
  </table>
</center>


## Description
This environmnet is similar to the classic inverted pendulum problem in control theory. ....


## Action Space
The action shape is `(1,)` with values between `-1` and `1` corresponding to the torque applued to the pendulum to make it swing.


## Observation Space
The observation is a ndarray with shape `(5,)` with the values between `-2` and `2` corresponding to ....


## Rewards
The reward is the negative normalized value of the euclidean distance between the pendulum's tip and the top upright point where the tip should reach and stabilize.


## Starting State
The episode starts with the pendulum at a random angle rotating with a random velocity.

## Arguments
This environment currently has no additional arguments.

```python
import gym
import sofagym
from sofagym.envs import *

gym.make('stempendulum-v0')
```


## Episode End
The episode ends if any one of the following occurs:
- Termination: 
  1. ....
- Truncation (when using the time_limit wrapper): 
  1. The length of the episode reaches the limit specified using the `TimeLimit` wrapper.


## Version History
v0: Initial versions release