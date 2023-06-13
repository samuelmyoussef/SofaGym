# CartStem


<center>
    <img src="../../../images/cartstem-v0.png" width="500"/>

  <table>
    <tr>
      <td><b>Action Space</b></td>
      <td>Discrete(2)</td>
    </tr>
    <tr>
      <td><b>Observation Space</b></td>
      <td>Box([-100]*4, [100]*4, (4,), float32)</td>
    </tr>
    <tr>
      <td><b>Import</b></td>
      <td>gym.make("cartstem-v0")</td>
    </tr>
  </table>
</center>


## Description
This environment is a soft equivalent to the cart-pole example, where the rigid pole is replaced by a stem which is flexible beam with a sphere at its end. This flexible pendulum is unactuated and attached to the cart, which moves on a frictionless surface. The goal is to balance the stem and the sphere upright by moving the cart left and right.


## Action Space
The action is discrete which can take values `{0, 1}` corresponding to the direction in which the cart is pushed by applying a fixed force to it.

- 0: Push cart to the right
- 1: Push cart to the left

## Observation Space
The observation is a ndarray with shape `(4,)` with the values between `-100` and `100` corresponding to the x positions and velocities of the cart and the sphere.

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
      <td>-100</td>
      <td>100</td>
    </tr>
    <tr>
      <td>1</td>
      <td>Sphere X Position</td>
      <td>-100</td>
      <td>100</td>
    </tr>
    <tr>
      <td>2</td>
      <td>Cart Velocity</td>
      <td>-100</td>
      <td>100</td>
    </tr>
    <tr>
      <td>3</td>
      <td>Sphere Velocity</td>
      <td>-100</td>
      <td>100</td>
    </tr>
  </table>
</center>


## Rewards
The reward is the absolute difference in the x position between the cart and the sphere.

$$
r = abs(cart\_position - sphere\_position)
$$


## Starting State
The episode starts with the cart in its initial position and the pendulum in the upright position.


## Arguments
This environment currently has no additional arguments.

```python
import gym
import sofagym
from sofagym.envs import *

gym.make('cartstem-v0')
```


## Episode End
The episode ends if any one of the following occurs:
- Termination: 
  1. ....
- Truncation (when using the time_limit wrapper): 
  1. The length of the episode reaches the limit specified using the `TimeLimit` wrapper.


## Version History
v0: Initial versions release