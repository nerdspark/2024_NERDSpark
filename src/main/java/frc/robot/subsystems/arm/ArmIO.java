// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs {
        public double shoulderLeftPosition = 0.0;
        public double shoulderLeftVelocity = 0.0;
        public double shoulderLeftAppliedVolts = 0.0;
        public double[] shoulderLeftCurrentAmps = new double[] {};

        public double elbowLeftPosition = 0.0;
        public double elbowLeftVelocity = 0.0;
        public double elbowLeftAppliedVolts = 0.0;
        public double[] elbowLeftCurrentAmps = new double[] {};

        public double shoulderRightPosition = 0.0;
        public double shoulderRightVelocity = 0.0;
        public double shoulderRightAppliedVolts = 0.0;
        public double[] shoulderRightCurrentAmps = new double[] {};

        public double elbowRightPosition = 0.0;
        public double elbowRightVelocity = 0.0;
        public double elbowRightAppliedVolts = 0.0;
        public double[] elbowRightCurrentAmps = new double[] {};


        public double gripperPosition = 0.0;
        public double gripperVelocity = 0.0;
        public double gripperAppliedVolts = 0.0;
        public double[] gripperCurrentAmps = new double[] {};

        public double armX = 0.0;
        public double armY = 0.0;
    }

    default void updateInputs(ArmIOInputs inputs) {}

    default void setGains(boolean climbing) {}

    default void setArmVelocity(Translation2d velocity) {}

    default void setArmPosition(Translation2d position, boolean inBend) {}

    default void resetEncoders() {}

    default Translation2d getArmPosition() {
        return new Translation2d();
    }

    default void setShoulderPosition(double position) {}

    default void setGripperPower(double power) {}

    default double getShoulderLeftPosition() {
        return 0.0;
    }

    default double getShoulderRightPosition() {
        return 0.0;
    }

    default void setElbowPosition(double position) {}

    default double getElbowLeftVelocity() {
        return 0.0;
    }

    default double getShoulderLeftVelocity() {
        return 0.0;
    }

    default double getElbowRightVelocity() {
        return 0.0;
    }

    default double getShoulderRightVelocity() {
        return 0.0;
    }

    default double getElbowLeftPosition() {
        return 0.0;
    }

    default double getElbowRightPosition() {
        return 0.0;
    }

    default double getGripperPosition() {
        return 0.0;
    }

}
