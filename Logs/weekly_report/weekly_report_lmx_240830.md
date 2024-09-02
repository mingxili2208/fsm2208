# LIMINGXI weekly_report in 2024/08/26--2024/08/30

## Planned

Complete single-module testing and realize the acceptance of the combined effect of testing and real vehicle joint debugging.

## Finished

Completed the following unit module testing:

1. NRF24L01 RF chip mounted on STM32F10x chip.
2. Instruction forwarding module controlled by Arduino microcontroller.
3. NRF communication protocol, encoding and decoding.
4. Remote control processing unit for remote control car based on NRF.
5. Joint debugging and acceptance of the above modules with the remote control car (expected effect: to achieve different motion states of the remote control car by adjusting the instruction field content in the communication packet).

## Unexpected events

### Problem:
The control logic of the remote control car does not meet expectations.

### Problem Description:
During the test, it was found that the motion control logic of the remote control car is: instruct the steering gear to rotate to the specified **angle**.
However, under normal circumstances, when dealing with Ackermann-type vehicles, we give an **angular velocity** parameter for the steering gear rotation to achieve a more real-time control effect.

### Preliminary Solution:

* Option 1: Rewrite the motion control logic of the remote control car to accept angular velocity commands.
  The difficulty with this solution is that the rewritten control scheme may not be stable enough.
* Option 2: Rewrite the autoware motion commands to give angle commands.
  The difficulty with this solution is that the angle command control method will increase the complexity of control, increase the error, and take longer to implement.

The selection and implementation of both solutions require studying the PID control code of the remote control car and the navigation stack code of Autoware.
Therefore, next week will mainly focus on the in-depth study of this aspect.

## Next work plan

1. Study the installed Autoware source code, find the navigation stack configuration plan, and design how to introduce tracker coordinates.
2. Study the PID control code of the remote control car, implement modifications to the control logic, and solve the above problems.