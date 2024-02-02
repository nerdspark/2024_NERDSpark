// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class Constants {
    public final class ArmConstants {
        public static final double baseStageLength = 24; // inches
        public static final double secondStageLength = 18; // inches

        public static final double shoulderRadPerRot = 2 * Math.PI;
        public static final double elbowRadPerRot = 2 * Math.PI;
        public static final double wristRadPerRot = 2 * Math.PI;

        public static final double shoulderOffset = 0; // radians, fwd = 0
        public static final double elbowOffset = 0;

        public static final Translation2d armBasePosition = new Translation2d();
        public static final double armForwardLimit = Units.inchesToMeters(12 + 5);
        public static final double armBackwardLimit = Units.inchesToMeters(12 + 28 - 5);
        public static final double armTopLimit = Units.inchesToMeters(48 - 8 - 4);

        public static final double virtual4BarGearRatio = 1.5;

        public static final class ArmSetPoints {
            public static final Translation2d home = new Translation2d();
            public static final Translation2d pickup = new Translation2d();
            public static final Translation2d amp = new Translation2d();
        }
    }

    public static final int intakeMotorId = 0;
    public static final int deployMotorId = 0;
    public static final int shooterMotor2ID = 0;
    public static final int shooterMotor1ID = 0;
    public static final int anglemotorID = 0;

    public static final int shoulderLeftID = 0;
    public static final int shoulderRightID = 0;
    public static final int elbowLeftID = 0;
    public static final int elbowRightID = 0;
    public static final int wristID = 0;
    public static final int gripperID = 0;

    public static final Mode currentMode = Mode.REAL;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }
}
