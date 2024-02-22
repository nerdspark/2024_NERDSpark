// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs {
        public static double shoulderLeftPosition = 0.0;
        public double shoulderLeftVelocity = 0.0;
        public double shoulderLeftAppliedVolts = 0.0;
        public double[] shoulderLeftCurrentAmps = new double[] {};

        public double elbowLeftPosition = 0.0;
        public double elbowLeftVelocity = 0.0;
        public double elbowLeftAppliedVolts = 0.0;
        public double[] elbowLeftCurrentAmps = new double[] {};

        public static double shoulderRightPosition = 0.0;
        public double shoulderRightVelocity = 0.0;
        public double shoulderRightAppliedVolts = 0.0;
        public double[] shoulderRightCurrentAmps = new double[] {};

        public double elbowRightPosition = 0.0;
        public double elbowRightVelocity = 0.0;
        public double elbowRightAppliedVolts = 0.0;
        public double[] elbowRightCurrentAmps = new double[] {};

        public double wristPosition = 0.0;
        public double wristVelocity = 0.0;
        public double wristAppliedVolts = 0.0;
        public double[] wristCurrentAmps = new double[] {};

        // public double gripperPosition = 0.0;
        // public double gripperVelocity = 0.0;
        // public double gripperAppliedVolts = 0.0;
        // public double[] gripperCurrentAmps = new double[] {};
    }

    default void setArmPosition(Translation2d position, boolean inBend) {}

    default void resetEncoders() {}

    /** Creates a new ArmIO. */
    default void updateInputs(ArmIOInputs inputs) {}

    default void setShoulderPosition(double position) {}

    default void setShoulderVelocity(double velocity) {}

    default double getShoulderPosition() {
        return 0;
    }

    default void setElbowPosition(double position) {}

    default void setElbowVelocity(double velocity) {}

    default double getElbowPosition() {
        return 0;
    }

    default void setWristPosition(double position) {}

    default double getWristPosition() {
        return 0;
    }

    default void setGripper(double power) {}

    default Translation2d getArmPosition() {
        return new Translation2d();
    }
}
