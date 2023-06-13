# CartStemContact


<center>
    <img src="../../../images/cartstemcontact-v0.png" width="500"/>


  <table>
    <tr>
      <td><b>Action Space</b></td>
      <td>Box([-1], [1], (1,), float32)</td>
    </tr>
    <tr>
      <td><b>Observation Space</b></td>
      <td>Box([-1]*8, [1]*8, (8,), float32)</td>
    </tr>
    <tr>
      <td><b>Import</b></td>
      <td>gym.make("cartstemcontact-v0")</td>
    </tr>
  </table>
</center>


## Description
This environemnt is similar to the `cartstem` environment. The pendulum is positioned between two box contacts and the aim is to make the stem bend due to its contact with the corner of one of the boxes in a way to make the sphere reach the red goal line.


## Action Space
The action shape is `(1,)` which can take a value between `-1` and `1` corresponding to the position of the cart in the x direction. The `0` position is exactly in the middle between the two contacts, negative and positive values correspond to the left and right, respectively.


## Observation Space
The observation is a ndarray with shape `(8,)` with the values between `-1` and `1` corresponding to the x positions of the cart, the sphere, the two contacts, and the goal, and the 3D dimensions of the boxes.

<center>
  <table>
    <tr>
      <th>Num</th>
      <th>Observation</th>
      <th>Min</th>
      <th>Max</th>
    </tr>
    <tr>
      <td>0</td>
      <td>Cart X Position</td>
      <td>-1</td>
      <td>1</td>
    </tr>
    <tr>
      <td>1</td>
      <td>Sphere X Position</td>
      <td>-1</td>
      <td>1</td>
    </tr>
    <tr>
      <td>2</td>
      <td>Contact Box 1 X Position</td>
      <td>-1</td>
      <td>1</td>
    </tr>
    <tr>
      <td>3</td>
      <td>Contact Box 2 X Position</td>
      <td>-1</td>
      <td>1</td>
    </tr>
    <tr>
      <td>4</td>
      <td>Contact Boxes X Dimension</td>
      <td>-1</td>
      <td>1</td>
    </tr>
    <tr>
      <td>5</td>
      <td>Contact Boxes Z Dimension</td>
      <td>-1</td>
      <td>1</td>
    </tr>
    <tr>
      <td>6</td>
      <td>Contact Boxes Y Dimension</td>
      <td>-1</td>
      <td>1</td>
    </tr>
    <tr>
      <td>7</td>
      <td>Goal X Position</td>
      <td>-1</td>
      <td>1</td>
    </tr>
  </table>
</center>


## Rewards
The reward is the negative normalized value of the absolute difference in the x position between the sphere and the goal. It has a value between -1 and 0 for each step. The maximum allowed distance used for normalization is set using the env config parameter `max_move`.


## Starting State
The episode starts with the cart in its initial position in the middle between the two box contacts and the pendulum is upright. The goal line is initialized at a random position within the workspace.


## Arguments
This environment currently has no additional arguments.

```python
import gym
import sofagym
from sofagym.envs import *

gym.make('cartstemcontact-v0')
```


## Episode End
The episode ends if any one of the following occurs:
- Termination: 
  1. The sphere reaches the goal line.
- Truncation (when using the time_limit wrapper): 
  1. The length of the episode reaches the limit specified using the `TimeLimit` wrapper.


## Version History
v0: Initial versions release