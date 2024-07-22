# the difference between autoware and apollo

## previous

## architecture

apollo

![h1](img/2024-07-22-17-36-30.png)

autoware

![h2](img/2024-07-22-17-37-16.png)

### software

1. autoware using ros as Middleware.

    ROS kernel scheduling does not have any business logic.

2. apollo using CyberRT as Middleware.
   ![S-1](img/2024-07-22-16-51-17.png)

    using coroutine Realize close integration of scheduling and algorithm business logic

3. ros2 provide DDS as coroutine

## safety
