// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotIdentity;

/** Add your docs here. */
public final class Constants {
    public final class ArmConstants {
        public static final double baseStageLength = 19; // inches
        public static final double secondStageLength = 17; // inches

        public static final double shoulderRadPerRot = 2 * Math.PI/15 * 14/32;
        public static final double elbowRadPerRot = 2 * Math.PI/4 * 18/42;
        public static final double wristRadPerRot = 2 * Math.PI;

        public static final double shoulderOffset = -0.17778; // radians, fwd = 0
        public static final double elbowOffset = 2.71796;

        public static final Translation2d armBasePosition = new Translation2d();
        public static final double armForwardLimit = Units.inchesToMeters(12 + 5);
        public static final double armBackwardLimit = Units.inchesToMeters(12 + 28 - 5);
        public static final double armTopLimit = Units.inchesToMeters(48 - 8 - 4);

        public static final double virtual4BarGearRatio = 1.5;

        public static final double maxPowerShoulder = 0.3;
        public static final double maxPowerElbow = 0.3;
        public static final int currentLimitShoulder = 32; // 32/20
        public static final int currentLimitElbow = 20;
        public static final double rampRateShoulder = 0.1;
        public static final double rampRateElbow = 0.1;

        public static final double shoulderP = 0.5; 
        public static final double shoulderI = 0.0; 
        public static final double shoulderD = 0.0087; 
        public static final double elbowP = 0.2; 
        public static final double elbowI = 0.0; 
        public static final double elbowD = 0.0049; 
        public static final double shoulderS = 0.0; // feedforward
        public static final double shoulderG = 0.012;
        public static final double shoulderV = 9e-6; 
        public static final double shoulderA = 0.0;
        public static final double elbowS = 0.0;
        public static final double elbowG = 0.01;
        public static final double elbowV = 8e-7; 
        public static final double elbowA = 0.0;


        public static final class ArmSetPoints {
            public static final Translation2d home = new Translation2d(0,36);//A
            public static final Translation2d pickup = new Translation2d(25,25);//B
            public static final Translation2d amp = new Translation2d(36,0);//X
        }
    }

    public static final int intakeMotorId = 0;
    public static final int deployMotorId = 0;
    public static final int shooterMotor2ID = 0;
    public static final int shooterMotor1ID = 0;
    public static final int anglemotorID = 0;

    public static final int shoulderLeftID = 9;
    public static final int shoulderRightID = 0;
    public static final int elbowLeftID = 8;
    public static final int elbowRightID = 0;
    public static final int wristID = 0;
    public static final int gripperID = 0;

    public static final Mode currentMode = Mode.REAL;
    public static final RobotIdentity compRobot = RobotIdentity.COMPETITION_ROBOT_2024;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }
}
