# Project: 3D Control for Quadcopter

---

## Task:
Complete all 3 scenarios.



### Testing it Out aka Scenario 1.

This is the test scenario to get familiar with the code and make sure everything is configured correctly. Changes made to the paramters file are:
```
QuadControlParams.Mass = 0.50 kg
QuadControlParams.Mass * 9.81 / 4
```
![Scenario 1 Screenshot 1](./Scenario1.png)
### Body rate and roll/pitch control aka Scenario 2.

1. Implemented the following functions:

 - `GenerateMotorCommands()`: The main goal of this function is to compute the thrust for each rotor. To do so I tried to follow the lecture notes and translated the python code into C++.
```
    float f_t = collThrustCmd;
    float tau_x = momentCmd.x;
    float tau_y = momentCmd.y;
    float tau_z = momentCmd.z;

    //Perpecdicular distance to axis
    float l = L / 2*sqrt(2.0); 

    cmd.desiredThrustsN[0] = CONSTRAIN(0.25f * (f_t + (1.0f / l) * tau_x + (1.0f / l) * tau_y - (1.0f / kappa) * tau_z), minMotorThrust, maxMotorThrust); // front left
    cmd.desiredThrustsN[1] = CONSTRAIN(0.25f * (f_t - (1.0f / l) * tau_x + (1.0f / l) * tau_y + (1.0f / kappa) * tau_z), minMotorThrust, maxMotorThrust); // front right
    cmd.desiredThrustsN[2] = CONSTRAIN(0.25f * (f_t + (1.0f / l) * tau_x - (1.0f / l) * tau_y + (1.0f / kappa) * tau_z), minMotorThrust, maxMotorThrust); // rear left
    cmd.d
```
 - `BodyRateControl()`: Contrses a proportional controller  to compute p, q and r. The metodology to do it is the same as in the lecture compute error and multiply by proportional constant,

Additionally `kpPQR` in `QuadControlParams.txt` was tuned to control overshoot to its proper value.

![Scenario 2 Screenshot 2](./Scenario2.png)

### Position/velocity and yaw angle control aka Scenario 3

  - Implemented the function `LateralPositionControl()`:
  ```
    velCmd.x = CONSTRAIN(velCmd.x, -maxSpeedXY, maxSpeedXY);
  velCmd.y = CONSTRAIN(velCmd.y, -maxSpeedXY, maxSpeedXY);

  accelCmd = kpPosXY * (posCmd - pos)
      + kpVelXY * (velCmd - vel)
      + accelCmd;

  accelCmd.x = CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
  accelCmd.y = CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);
  ```
  velZCmd = CONSTRAIN(velZCmd, -maxAscentRate, maxDescentRate);

  const float bz = R(2, 2);
  const float errorPos = posZCmd - posZ;
  const float errorVel = velZCmd - velZ;
  integratedAltitudeError += errorPos * dt;

  const float u1Bar = kpPosZ * errorPos
      + kpVelZ * errorVel
      + KiPosZ * integratedAltitudeError
      + accelZCmd;
  const float c = (u1Bar - CONST_GRAVITY) / bz;

  thrust = -mass * CONSTRAIN(c, -maxAscentRate / dt, maxAscentRate / dt);;
  ```

  Additionally the parameters `kpPosZ`, `kpPosZ`, `kpVelXY` and `kpVelZ` were tuneer accordingly.

  ![Scenario 3 Screenshot 3](./scenario3.png)

### References
https://github.com/Amay22/Flying-Car-Quadcopter-Controller
https://github.com/ebaracaglia/Flying-Car-Nanodegree-Project3-Quadrotor-3D-Control