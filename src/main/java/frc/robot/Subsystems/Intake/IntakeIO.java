// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public static double m1_PositionRad = 0.0;
        public double m1_VelocityRadPerSec = 0.0;
        public double m1_AppliedVolts = 0.0;
        public double[] m1_CurrentAmps = new double[] {};

        public double m2_PositionRad = 0.0;
        public double m2_VelocityRadPerSec = 0.0;
        public double m2_AppliedVolts = 0.0;
        public double[] m2_CurrentAmps = new double[] {};
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(IntakeIOInputs inputs) {}

    /** Set to Start Position */
    default void moveArmToStartPosition() {}

    /** Move Arm to Other side */
    default void moveArmToOtherSide() {}

    /** Move Arm to Other side */
    default void moveArmToTestPosition() {}
}
