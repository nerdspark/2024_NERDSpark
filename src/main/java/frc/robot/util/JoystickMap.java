// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class JoystickMap {

    // Joystick map constants: (1:1 for start, can be changed later)
    public static final double[] joystickIncrement = {0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1};
    public static final double[] joystickPowerOutput = {0, 0.075, 0.1, 0.15, 0.2, 0.35, 0.5, 0.65, 0.8, 0.9, 1};

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
