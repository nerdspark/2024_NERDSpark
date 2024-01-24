// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

/** Add your docs here. */
public class JoystickMap {

    // Joystick map constants: (1:1 for start, can be changed later)
    public static final double[] joystickIncrement =   {0.00, 0.03, 0.04, 0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 1};
    public static final double[] joystickPowerOutput = {0.00, 0.00, 0.02, 0.04, 0.08, 0.12, 0.17, 0.22, 0.26, 0.36, 0.60, 0.80, 1.00, 1};

    public static double JoystickPowerCalculate(double joystickReading) {

        double power = 0; // power output from robot (-1 to 1)

   

        int low = 0, high = joystickIncrement.length - 1; // find where joystickReading is inbetween array values
        while (high - low > 1) {
            int mid = (low + high) / 2;
            if (joystickIncrement[mid] > Math.abs(joystickReading)) {

                high = mid;
            } else {
                low = mid;
            }
        }

        double joystickIncrementSlope = (joystickPowerOutput[high] - joystickPowerOutput[low])
                / (joystickIncrement[high] - joystickIncrement[low]);
        power = joystickIncrementSlope * (Math.abs(joystickReading) - joystickIncrement[low])
                + joystickPowerOutput[low]; // calculate necesary power output

        if (joystickReading < 0) {
            power = -power;
        }

        return power;
    }
}
