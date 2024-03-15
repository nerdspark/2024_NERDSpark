// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    /** Creates a new ClimbIO. */
    @AutoLog
    class ClimbIOInputs {
        public static double climbPosition = 0.0;
        public double climbVelocity = 0.0;
        public double climbAppliedVolts = 0.0;
        public double[] climbCurrentAmps = new double[] {};

    }

    default void updateInputs(ClimbIOInputs inputs) {}
}
