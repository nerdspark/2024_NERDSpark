package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.RobotIdentity;
import frc.robot.util.Alert;

public final class Constants {

    public static final boolean PracticeBot =
            false; // SMIDGE true; SMUDGE false TODO TODO TODO TODO TODO CHANGHACANHEHNCHANGE


    public final class FourBarGains {
        public static final double kP = 0.95; // 0.65; // 0.85
        public static final double kI = 0.001; // 0; // 0.2
        public static final double kD = 0.10; // 0.006; // 0.01
        public static final double kIZone = 0.2;

        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kG = 0.035;
        public static final double kA = 0.0;
    }

    public final class FixedShotConstants {
        public static final double fourBarLong = 1.6; // TODO T UNE
        // public static final double fourBarPodium = 1.95; //TODO tune

        public static final double RPMLong = 5200.0;
        // public static final double RPMPodium = 3000.0;
        public static final double RPMPointBlank = 3500.0;
        public static final double RPMHome = 4300.0;
    }

    public final class FourBarConstants {
        public static final int currentLimit = 70;
        public static final double closedLoopRampRate = 0.1; //0.25, 0.1
        public static final double openLoopRampRate = 0.1; //0.15, 0.1
        public static final double positionConversionFactor = 2.0 * Math.PI * 1 / (56d / 18d * 25d);
        public static final double resetPosition = Math.PI - 0.9948; // zero position from CAD
        public static final double fourBarOut = 0.52;
        public static final double fourBarHome = 2.148;
        public static final double fourBarTolerance = 0.015;
        public static final double fourBarHotel = 88;
        public static final IdleMode fourBarIdleMode = IdleMode.kBrake;

        public static InterpolatingDoubleTreeMap fourBarMap = new InterpolatingDoubleTreeMap();

        static {
            // Key: Distance in feet
            // Value: Shooter Position
            // fourBarMap.put(4.0, 2.08);
            // fourBarMap.put(4.6, 2.02d);
            // fourBarMap.put(4.8, 1.93d);
            // fourBarMap.put(4.89, 1.99d);
            // fourBarMap.put(5.06, 1.99d);
            // fourBarMap.put(5.15, 2d);

            fourBarMap.put(1.6, 0.52);
            fourBarMap.put(1.75, 0.52);
            fourBarMap.put(1.96, 0.56);
            fourBarMap.put(2.14, 0.62);
            fourBarMap.put(2.35, 0.67);
            fourBarMap.put(3.19, fourBarHotel);
            fourBarMap.put(3.1999999999, fourBarHotel);

            fourBarMap.put(3.2, fourBarHome);

            fourBarMap.put(3.3, fourBarHome);
            fourBarMap.put(3.31, fourBarHome - 0.1); // 4barhome un-backlash
            fourBarMap.put(4.08, 2.08 - 0.1);
            fourBarMap.put(4.45, 2.07 - 0.1);
            fourBarMap.put(5.0, 2.05 - 0.1);
            fourBarMap.put(5.53, 1.97 - 0.1);
            fourBarMap.put(6.44, FixedShotConstants.fourBarLong);
            // fourBarMap.put(4.75 + 0.2, 1.925d);
            // fourBarMap.put(4.85 + 0.2, 1.925d);
            // fourBarMap.put(4.90 + 0.2, 1.925);
            // fourBarMap.put(5.00 + 0.2, 1.85);
            // fourBarMap.put(5.10 + 0.2, 1.9);
            // fourBarMap.put(5.30 + 0.2, 1.79);
            // fourBarMap.put(5.40 + 0.15, 1.5);
            fourBarMap.put(15.1, FixedShotConstants.fourBarLong);

            /* OLD POSITIONS W/O NEW CONVERSION FACTOR */
            // fourBarMap.put(49.5, 8.0);
            // fourBarMap.put(25.5, 8.0);
            // fourBarMap.put(19.5, 5.4); // 6.7
            // fourBarMap.put(17.5, 2.84); // 4.2
            // fourBarMap.put(15.5 + (2.0 / 12.0), 2.37);
            // fourBarMap.put(13.5 + (1.0 / 12.0), 1.80);
            // fourBarMap.put(13.0, 1.65);
            // fourBarMap.put(10.5 + (9.0 / 12.0), 1.3);
            // fourBarMap.put(6.5, 0.8);
            // fourBarMap.put(6.4, 18.0);
            // fourBarMap.put(5.4, 20.0);
            // fourBarMap.put(3.0, 20.0);

            /* OLD POSITIONS WITH NEW CONVERSION FACTOR */
            // fourBarMap.put(15.091, 1.500);
            // fourBarMap.put(7.774, 1.500);
            // fourBarMap.put(5.945, 1.710);
            // fourBarMap.put(5.335, 1.917);
            // fourBarMap.put(4.786, 1.955);
            // fourBarMap.put(4.146, 2.001);
            // fourBarMap.put(3.963, 2.013);
            // fourBarMap.put(3.445, 2.041);
            // fourBarMap.put(1.981, 2.082);
            // fourBarMap.put(1.951, 0.692);
            // fourBarMap.put(1.646, 0.531);
            // fourBarMap.put(0.914, 0.531);
        }
    }

    public final class ArmConstants {
        public static final double baseStageLength = 18.75; // inches
        public static final double secondStageLength = 16.975; // inches

        public static final double virtual4BarGearRatio = 36.0 / 42.0;
        public static final double shoulderRadPerRot = 2.0 * Math.PI / 125.0 * 14.0 / 32.0;
        public static final double elbowRadPerRot = 2.0 * Math.PI / 64.0 * virtual4BarGearRatio;
        public static final double wristRadPerRot = 2.0 * Math.PI / 27.0;

        public static final double shoulderOffset = -0.144; // radians, fwd = 0
        public static final double elbowOffset = 2.611; // negative of measurement
        public static final double wristOffset = 0.0;

        public static final Translation2d armBasePosition = new Translation2d();
        public static final double armForwardLimit = Units.inchesToMeters(12 + 5);
        public static final double armBackwardLimit = Units.inchesToMeters(12 + 28 - 5);
        public static final double armTopLimit = Units.inchesToMeters(48 - 8 - 4);

        public static final double maxPowerShoulder = 0.3;
        public static final double maxPowerElbow = 0.3;
        public static final double maxPowerWrist = 0.35;
        public static final int currentLimitShoulder = 60;
        public static final int currentLimitElbow = 60;
        public static final double rampRateShoulder = 0.1;
        public static final double rampRateElbow = .1;

        public static class ArmGains {
            private final double shoulderP = 2.0;
            private final double shoulderI = 0.0;
            private final double shoulderD = 0.0;
            private final double elbowP = 0.7;
            private final double elbowI = 0.07;
            private final double elbowD = 0.0;
            private final double shoulderS = 0.0;
            private final double shoulderG = 0.015;
            private final double shoulderV = 0.0;
            private final double shoulderA = 0.0;
            private final double elbowS = 0.0;
            private final double elbowGLeft = .03;
            private final double elbowGRight = .03;
            private final double elbowV = 0.0;
            private final double elbowA = 0.0;
            public static final double wristP = 0.4;
            public static final double wristI = 0.0;
            public static final double wristD = 0.0;

            public final PIDController shoulderLeftController = new PIDController(shoulderP, shoulderI, shoulderD);
            public final PIDController shoulderRightController = new PIDController(shoulderP, shoulderI, shoulderD);
            public PIDController elbowLeftController = new PIDController(elbowP, elbowI, elbowD);
            public PIDController elbowRightController = new PIDController(elbowP, elbowI, elbowD);

            public final ArmFeedforward shoulderLeftFeedforward =
                    new ArmFeedforward(shoulderS, shoulderG, shoulderV, shoulderA);
            public final ArmFeedforward shoulderRightFeedforward =
                    new ArmFeedforward(shoulderS, shoulderG, shoulderV, shoulderA);
            public final ArmFeedforward elbowLeftFeedforward = new ArmFeedforward(elbowS, elbowGLeft, elbowV, elbowA);
            public final ArmFeedforward elbowRightFeedforward = new ArmFeedforward(elbowS, elbowGRight, elbowV, elbowA);
        }

        public static class ArmGainsClimb extends ArmGains {
            private final double shoulderP = 2.3;
            private final double shoulderI = 0.001;
            private final double shoulderD = 0.0;
            private final double elbowP = 0.3;
            private final double elbowI = 0.01;
            private final double elbowD = 0.0;
            private final double shoulderS = 3.0;
            private final double shoulderG = -0.5;
            private final double shoulderV = 0.0;
            private final double shoulderA = 0.0;
            private final double elbowS = 1.0;
            private final double elbowG = 0.0;
            private final double elbowV = 0.0;
            private final double elbowA = 0.0;

            public final PIDController shoulderLeftController = new PIDController(shoulderP, shoulderI, shoulderD);
            public final PIDController shoulderRightController = new PIDController(shoulderP, shoulderI, shoulderD);
            public PIDController elbowLeftController = new PIDController(elbowP, elbowI, elbowD);
            public PIDController elbowRightController = new PIDController(elbowP, elbowI, elbowD);

            public final ArmFeedforward shoulderLeftFeedforward =
                    new ArmFeedforward(shoulderS, shoulderG, shoulderV, shoulderA);
            public final ArmFeedforward shoulderRightFeedforward =
                    new ArmFeedforward(shoulderS, shoulderG, shoulderV, shoulderA);
            public final ArmFeedforward elbowLeftFeedforward = new ArmFeedforward(elbowS, elbowG, elbowV, elbowA);
            public final ArmFeedforward elbowRightFeedforward = new ArmFeedforward(elbowS, elbowG, elbowV, elbowA);
        }

        public static final class ArmSetPoints {
            public static final Translation2d home = new Translation2d(
                            baseStageLength * Math.cos(shoulderOffset), baseStageLength * Math.sin(shoulderOffset))
                    .plus(new Translation2d(
                            secondStageLength * Math.cos(elbowOffset), secondStageLength * Math.sin(elbowOffset))); // A
            public static final double homeWrist = 0.0;
            public static final Translation2d pickup = new Translation2d(
                            baseStageLength * Math.cos(shoulderOffset), baseStageLength * Math.sin(shoulderOffset))
                    .plus(new Translation2d(
                            secondStageLength * Math.cos(elbowOffset - Units.degreesToRadians(2.5)),
                            secondStageLength * Math.sin(elbowOffset - Units.degreesToRadians(2.5)))); // B
            public static final double pickupWrist = 1.95;
            public static final Translation2d amp = new Translation2d(-1, 16); // X
            public static final double ampWrist = pickupWrist;
            public static final Translation2d dropoff = new Translation2d(0, 27.5); // Y
            public static final double dropoffWrist = -0.3;
            public static final double dropoffMultiplier = 7.4;
            public static final double dropoffMultiplierY = 3;
        }

        public static final class ClimbSetPoints {
            public static final double readyShoulder = Units.degreesToRadians(100);
            public static final double downElbow = Units.degreesToRadians(-270);
            // public static final double pinchShoulder = Units.degreesToRadians(60);
            // public static final double pinchElbow = Units.degreesToRadians(-50);
            public static final double forwardShoulder = readyShoulder - Units.degreesToRadians(15.0);
            public static final Translation2d ready = new Translation2d(
                            baseStageLength * Math.cos(shoulderOffset + readyShoulder),
                            baseStageLength * Math.sin(shoulderOffset + readyShoulder))
                    .plus(new Translation2d(
                            secondStageLength * Math.cos(elbowOffset + readyShoulder),
                            secondStageLength * Math.sin(elbowOffset + readyShoulder))); // A
            public static final double readyWrist = 0.0;
            public static final Translation2d down = new Translation2d(
                            baseStageLength * Math.cos(shoulderOffset + forwardShoulder),
                            baseStageLength * Math.sin(shoulderOffset + forwardShoulder))
                    .plus(new Translation2d(
                            secondStageLength * Math.cos(elbowOffset + downElbow),
                            secondStageLength * Math.sin(elbowOffset + downElbow))); // B
            public static final double downWrist = Math.PI;
            public static final Translation2d pinch =
                    ArmSetPoints.home.rotateBy(new Rotation2d(Units.degreesToRadians(42)));
            // new Translation2d(
            //         baseStageLength * Math.cos(shoulderOffset + pinchShoulder),
            //         baseStageLength * Math.sin(shoulderOffset + pinchShoulder))
            // .plus(new Translation2d(
            //         secondStageLength * Math.cos(elbowOffset + pinchShoulder),
            //         secondStageLength * Math.sin(elbowOffset + pinchShoulder))); // X
            public static final double pinchWrist = 0.0;
            public static final Translation2d forward = new Translation2d(
                            baseStageLength * Math.cos(shoulderOffset + forwardShoulder),
                            baseStageLength * Math.sin(shoulderOffset + forwardShoulder))
                    .plus(new Translation2d(
                            secondStageLength * Math.cos(elbowOffset + forwardShoulder),
                            secondStageLength * Math.sin(elbowOffset + forwardShoulder))); // X
            public static final double forwardWrist = Math.PI;
            public static final Translation2d trap =
                    new Translation2d(0, baseStageLength + secondStageLength); // right stick
            public static final double trapwrist = Math.PI / 2;
            public static final double trapMultiplier = 3.0;
        }
    }

    public final class AutoConstants {
        public static final double redCenterRing2 = 1.95; // 2.5
        public static final double redCenterRing3 = 1.95; // 2.5
        public static final double redCenterRing4 = 1.95; // 1.9

        public static final double blueCenterRing2 = 1.95; // 2.7
        public static final double blueCenterRing3 = 1.95; // 2.45
        public static final double blueCenterRing4 = 1.97; // 1.9

        public static final double weirdSideRing2 = 1.87; // 3.5
        public static final double weirdSideRing3 = 1.87; // 2.75
        public static final double weirdSideRing4 = 2.148; // 1.9

        public static final double red_weirdSideRing1 = 3.2;
        public static final double red_weirdSideRing2 = 1.87;
        public static final double red_weirdSideRing3 = 1.87;
        public static final double red_weirdSideRing4 = 2.148;

        public static final double blueAmpSide1 = 2.0; // 2.7
        public static final double blueAmpSide2 = 1.87; // 2.45
        public static final double blueAmpSide3 = 1.87; // 2.45
        public static final double blueAmpSide4 = 1.97; // 1.9

        public static final double blueRECenterNote5 = 1.92; // 1.80
        public static final double blueRECenterNote6 = 1.95; // 1.87

        public static final double blueStarWars1 = 1.95; // 3.5
        public static final double blueStarWars2 = 1.87; // 2.75
        public static final double blueStarWars3 = 1.87;
        public static final double blueStarWars4 = 2.148;
    }

    public final class RobotMap {
        public static final int intakeMotorId = 4;
        // public static final int deployMotorId = 0;
        public static final int shooterMotor2ID = 7;
        public static final int shooterMotor1ID = 6;
        // public static final int anglemotorID = 0;

        public static final int fourBarLeftID = 2;
        public static final int fourBarRightID = 3;

        public static final int shoulderLeftID = 11;
        public static final int shoulderRightID = 9;
        public static final int elbowLeftID = 10;
        public static final int elbowRightID = 8;
        public static final int wristID = 5;
        // public static final int gripperID = 0;

        public static final int wristChannel1 = 0;
        public static final int wristChannel2 = 1;

        public static final int pigeonID = 25;
        // public static final double wristPulseDist = 8192.0 * 2.0 * Math.PI;
    }

    public final class DrivetrainConstants {
        public static final double gyroP = 0.023;
        public static final double gyroI = 0.0;
        public static final double gyroD = 0.0018;
        public static final double IZone = 10.0;

        public static final double autoTurnCeiling = 6.0;
        public static final double poseSyncTolerance =
                0.5; // the tolerance at which vision pose and estimated pose have to be in for driver station to report
        // happy
    }

    public static class VisionConstants {

        public static enum VisionDeviationDistanceStrategy {
            SMALEST_DISTANCE,
            AVERAGE_DISTANCE
        }

        public static boolean USE_VISION = true;
        public static boolean USE_FRONT_CAMERA = true;
        public static boolean USE_ADV_KIT_VISION = true;
        public static boolean MULTI_TAG_RESULT_ENABLED = true;

        public static VisionDeviationDistanceStrategy VISION_DEV_DIST_STRATEGY =
                VisionDeviationDistanceStrategy.AVERAGE_DISTANCE;

        /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
        public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;

        public static final double NOISY_DISTANCE_METERS = 6;

        public static boolean USE_BACK_LEFT_CAMERA = true;
        public static boolean USE_BACK_RIGHT_CAMERA = true;

        public static final String FRONT_CAMERA_NAME = "BackCamera"; // LEFT
        public static final String BACK_LEFT_CAMERA_NAME = "BackLeft"; // RIGHT
        public static final String BACK_RIGHT_CAMERA_NAME = "BackRight";
        public static final String NOTE_CAMERA_NAME = "NoteCamera";

        /**
         * Physical location of the left camera on the robot, relative to the center of the robot.
         */
        public static final Transform3d ROBOT_TO_FRONT_CAMERA = new Transform3d(
                new Translation3d(-Units.inchesToMeters(12.6), Units.inchesToMeters(0), Units.inchesToMeters(9)),
                new Rotation3d(0, Math.toRadians(0), Math.toRadians(180)));

        // public static final Transform3d ROBOT_TO_FRONT_CAMERA = new Transform3d(
        //         new Translation3d(Units.inchesToMeters(12.6), Units.inchesToMeters(0), Units.inchesToMeters(9)),
        //         new Rotation3d(180, -Math.toRadians(19.565), Math.toRadians(180)));

        /**
         * Physical location of the back camera on the robot, relative to the center of the robot.
         */
        public static final Transform3d ROBOT_TO_BACK_RIGHT_CAMERA = new Transform3d(
                new Translation3d(-Units.inchesToMeters(4), Units.inchesToMeters(-10.5), Units.inchesToMeters(8)),
                new Rotation3d(0, Math.toRadians(0), Math.toRadians(-60)));

        // Physical location of the back right camera on the robot, relative to the center of the robot. CHANGE THIS

        public static final Transform3d ROBOT_TO_BACK_LEFT_CAMERA = new Transform3d(
                new Translation3d(-Units.inchesToMeters(4), Units.inchesToMeters(10.5), Units.inchesToMeters(8)),
                new Rotation3d(0, Math.toRadians(0), Math.toRadians(60)));

        // PLACEHOLDER
        public static final double NOTE_CAMERA_HEIGHT_METERS = Units.inchesToMeters(13);
        public static final double NOTE_HEIGHT_METERS = Units.inchesToMeters(2);
        public static final double NOTE_CAMERA_PITCH_RADIANS = Units.degreesToRadians(40);
        // when fourbar up: 18 inches high, 26 deg

        // Vision Drive Constants

        public static final double TRANSLATION_TOLERANCE_X = 0.025; // Changed from 0.05 3/26/23
        public static final double TRANSLATION_TOLERANCE_Y = 0.025; // Changed from 0.05 3/26/23
        public static final double ROTATION_TOLERANCE = 0.035;

        public static final double MAX_VELOCITY = 3; // 3 //2
        public static final double MAX_ACCELARATION = 2; // 2 //1
        public static final double MAX_VELOCITY_ROTATION = 8; // 8
        public static final double MAX_ACCELARATION_ROTATION = 8; // 8

        public static final double kPXController = 2.5d;
        public static final double kIXController = 0d;
        public static final double kDXController = 0d;
        public static final double kPYController = 2.5d;
        public static final double kIYController = 0d;
        public static final double kDYController = 0d;
        public static final double kPThetaController = 1.2d;
        public static final double kIThetaController = 0d;
        public static final double kDThetaController = 0d;
    }

    public static class SpeakerConstants {
        public static final double speakerBlueY = Units.inchesToMeters(218.42);
        public static final double speakerRedY = Units.inchesToMeters(218.42);
        public static final double speakerBlueX = Units.inchesToMeters(4);
        public static final double speakerRedX = Units.inchesToMeters(652.73 - 4);

        public static final double speakerHeight = Units.inchesToMeters(80.515); // (82.90 + 78.13) / 2

        public static final Pose2d speakerLocBlue = new Pose2d(speakerBlueX, speakerBlueY, new Rotation2d(0));
        public static final Pose2d speakerLocRed = new Pose2d(speakerRedX, speakerRedY, new Rotation2d(Math.PI));

        public static final double autonAimDistanceThreshold = 2.0d;
    }

    public static class ShooterConstants {
        public static Measure<Distance> MAXIMUM_READYSHOOT_DISTANCE = Meters.of(Units.feetToMeters(15));

        public static double SHOOTER_SPEED = 10;

        public static final double CONSTANT_DISTANCE_ADD = 0.0; // ft

        public static InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();

        public static final double shootMoveMultiplier = 0.12; // theoretically speed of shot in m/s
        public static final double stillShotSpeed = 0.3;

        public static final double shooterTolerance = 100;

        static {
            // Key: Distance
            // Value: Shooter Position
            shooterMap.put(15.1, FixedShotConstants.RPMLong);
            shooterMap.put(6.44, FixedShotConstants.RPMLong);
            shooterMap.put(5.9436, 4900.0);
            shooterMap.put(5.334, 4800.0);
            shooterMap.put(4.7752, 4800.0);
            shooterMap.put(4.1402, 4700.0);
            shooterMap.put(3.429, 4500.0);
            shooterMap.put(2.7178, 4200.0);
            shooterMap.put(0.762, 3500.0);
        }
    }

    /* MISCELLANEOUS CONSTANTS */
    public static final double indexDistance = 1000;

    public static final Mode currentMode = Mode.REAL;
    public static final RobotIdentity compRobot = RobotIdentity.SMUDGE_2024;

    public static final int loopPeriodMs = 20;
    private static RobotType robotType = RobotType.COMPBOT;
    public static final boolean tuningMode = true;
    public static final boolean characterizationMode = false;

    public static RobotType getRobot() {
        if (RobotBase.isReal() && robotType == RobotType.SIMBOT) {
            new Alert("Invalid Robot Selected, using COMPBOT as default", Alert.AlertType.ERROR).set(true);
            robotType = RobotType.COMPBOT;
        }
        return robotType;
    }

    public static Mode getMode() {
        return switch (getRobot()) {
            case COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
            case SIMBOT -> Mode.SIM;
        };
    }

    public enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public enum RobotType {
        SIMBOT,
        COMPBOT
    }

    /** Checks whether the robot the correct robot is selected when deploying. */
    public static void main(String... args) {
        if (robotType == RobotType.SIMBOT) {
            System.err.println("Cannot deploy, invalid robot selected: " + robotType.toString());
            System.exit(1);
        }
    }
}
