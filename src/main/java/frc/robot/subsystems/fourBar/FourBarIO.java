// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.fourBar;

import org.littletonrobotics.junction.AutoLog;

public interface FourBarIO {

    @AutoLog
    class FourBarIOInputs {
        public static double FourBarPosition = 0.0;
        public double FourBarVelocity = 0.0;
        public double FourBarAppliedVolts = 0.0;
        public double[] FourBarCurrentAmps = new double[] {};
        public double FourBarTarget = 0.0;
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(FourBarIOInputs inputs) {}

    default void setFourBarAngle(double FourBarAngle) {}

    default void setPIDGGains(double kP, double kI, double kD, double kG) {}

    default double getFourBarAngle() {
        return 0;
    }
}
