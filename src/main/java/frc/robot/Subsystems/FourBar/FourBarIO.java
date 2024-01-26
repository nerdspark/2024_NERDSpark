// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.FourBar;

import frc.robot.Subsystems.FourBar.FourBarIO.FourBarIOInputs;
import org.littletonrobotics.junction.AutoLog;

public interface FourBarIO {

    @AutoLog
    class FourBarIOInputs {
        public static double FourBarPosition1 = 0.0;
        public double FourBarVelocity1 = 0.0;
        public double FourBarAppliedVolts1 = 0.0;
        public double[] FourBarCurrentAmps1 = new double[] {};

        public double FourBarPosition2 = 0.0;
        public double FourBarVelocity2 = 0.0;
        public double FourBarAppliedVolts2 = 0.0;
        public double[] FourBarCurrentAmps2 = new double[] {};
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(FourBarIOInputs inputs) {}

    default void setFourBarAngle(double FourBarAngle) {}

    default double getFourBarAngle() {
        return 0;
    }
}
