// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public static double intakePosition1 = 0.0;
        public double intakeVelocity1 = 0.0;
        public double intakeAppliedVolts1 = 0.0;
        public double[] intakeCurrentAmps1 = new double[] {};

        public double intakePosition2 = 0.0;
        public double intakeVelocity2 = 0.0;
        public double intakeAppliedVolts2 = 0.0;
        public double[] intakeCurrentAmps2 = new double[] {};
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(IntakeIOInputs inputs) {}

    default void setIntakePower(double intakePower) {}

    default double getIntakePower() {
        return 0;
    }

    default boolean getBeamBreak() {
        return false;
    }

    default double getIntakePosition() {
        return 0;
    }
}
