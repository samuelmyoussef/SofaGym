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
      <td>gym.make("trunk-v0")</td>
    </tr>
  </table>
</center>


## Description



## Action Space



## Observation Space



## Rewards



## Starting State



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
  1. 
- Truncation (when using the time_limit wrapper): 
  1. The length of the episode reaches the limit specified using the `TimeLimit` wrapper.


## Version History
v0: Initial versions release