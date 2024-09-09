# this is about how to transmit control cmd

the basic func we needed as blow

1. subscribe the topic "/control/command/control_cmd"
2. get speed and turn_angle
3. transmit to the correct range
4. package the command message follow the protocol R2A
5. send the command to the arduino transmitter
6. arduino get the command message and package the command message follow the protocol A2C
7. arduino send the package 
8. RC_CAR get msg through the NRF
9. get speed and turn_angle
10. set control