// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    /** Creates a new ClimbIO. */
    @AutoLog
    class ClimbIOInputs {
        public static double climbPosition1 = 0.0;
        public double climbVelocity1 = 0.0;
        public double climbAppliedVolts1 = 0.0;
        public double[] climbCurrentAmps1 = new double[] {};

        public double climbPosition2 = 0.0;
        public double climbVelocity2 = 0.0;
        public double climbAppliedVolts2 = 0.0;
        public double[] climbCurrentAmps2 = new double[] {};
    }

    default void updateInputs(ClimbIOInputs inputs) {}
}
