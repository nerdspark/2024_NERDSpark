// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class Constants {
    public final class AmrConstants {
        public static final double baseStageLength = 24; // inches
        public static final double secondStageLength = 18; // inches
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
