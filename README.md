# servo-stm32f0
NB:
NB: This software is a work in progress, it is not complete and will not function
NB:

RC Servo driver for model railroad turnouts

This program runs on an ST Microelectronics STM32F030 ARM processor. This micro has enough resources (pins,
timers, PWM outputs, etc.) to drive four RC servos and to provides support for the associated inputs and
outputs. For each servo the CPU provides one output pin to carry the servo's control signal, one input pin
to carry the state of a switch or button to control the position of the servo (and turnout) and two outputs
to drive feedback LEDs showing the servo/turnout position.

In addition there are two button inputs, BTNPLUS and BTNMINUS that can be used to increase or decrease the
range of motion of each servo so it can be tuned to properly position the turnout. These buttons act on the
most recently used servo on the board and the current position of that servo. Each press of BTNPLUS increases
the range of motion for that servo in the current direction and each press of BTNMINUS reduces the range of
motion in that direction.

This software can be used with the hardware defined in the github project
  https://github.com/RandyStockberger/????.git
or you can implement your own hardware and repurpose this software.
