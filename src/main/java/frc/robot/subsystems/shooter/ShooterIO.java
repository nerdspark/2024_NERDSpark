// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterIoInputs {
        public static double shooterPosition1 = 0.0;
        public double shooterVelocity1 = 0.0;
        public double shooterAppliedVolts1 = 0.0;
        public double[] shooterCurrentAmps1 = new double[] {};

        public double shooterPosition2 = 0.0;
        public double shooterVelocity2 = 0.0;
        public double shooterAppliedVolts2 = 0.0;
        public double[] shooterCurrentAmps2 = new double[] {};
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(ShooterIoInputs inputs) {}

    default void setSpeed(double speed1, double speed2) {}

    default double getShooterPower() {
        return 0;
    }
}
