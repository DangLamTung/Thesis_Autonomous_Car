# INS/GPS sensor fusion using EKF

This is my INS/ GPS sensor fusion library using STM32, this project is heavily inspired by the Ardupilot EKF

At this moment, this project use MPU 9250 and Ublox Max-8N on STM32F405RGT6, which has a single percision FPU and run at 168Mhz.

I implemented 2 EKF, one based on the ARM_MATH library and another is based on the equation generation like the Ardupilot EKF, the different is not significant, the ARM_MATH EKF run
in 125us and the Matlab is 150us.

These EKF library can be used with STM32F103C8T6 and STM32G030C8T6.
