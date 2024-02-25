package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.RobotIdentity;
import frc.robot.util.Alert;
import frc.robot.util.FieldConstants;
// import frc.robot.config.RobotIdentity;

public final class Constants {
    public final class ArmConstants {
        public static final double baseStageLength = 18.75; // inches
        public static final double secondStageLength = 16.975; // inches

        public static final double virtual4BarGearRatio = 36.0 / 42.0;
        public static final double shoulderRadPerRot = 2 * Math.PI / 15 * 14 / 32;
        public static final double elbowRadPerRot = 2 * Math.PI / 12 * virtual4BarGearRatio;
        public static final double wristRadPerRot = 2 * Math.PI / 13.5;

        public static final double shoulderOffset = -0.144; // radians, fwd = 0
        public static final double elbowOffset = 2.611; // negative of measurement

        public static final Translation2d armBasePosition = new Translation2d();
        public static final double armForwardLimit = Units.inchesToMeters(12 + 5);
        public static final double armBackwardLimit = Units.inchesToMeters(12 + 28 - 5);
        public static final double armTopLimit = Units.inchesToMeters(48 - 8 - 4);

        public static final double maxPowerShoulder = 0.3;
        public static final double maxPowerElbow = 0.3;
        public static final int currentLimitShoulder = 40;
        public static final int currentLimitElbow = 25;
        public static final double rampRateShoulder = 0.1;
        public static final double rampRateElbow = .1;

        public static final double shoulderP = 0.3;
        public static final double shoulderI = 0.0;
        public static final double shoulderD = 0.0;
        public static final double elbowP = 0.3;
        public static final double elbowI = 0.05;
        public static final double elbowD = 0.0;
        public static final double shoulderS = 0.0; // feedforward
        public static final double shoulderG = 0.012;
        public static final double shoulderV = 0.0;
        public static final double shoulderA = 0.0;
        public static final double elbowS = 0.0;
        public static final double elbowG = .027;
        public static final double elbowV = 0.0;
        public static final double elbowA = 0.0;

        public static final class ArmSetPoints {
            public static final Translation2d home = new Translation2d(
                            baseStageLength * Math.cos(shoulderOffset), baseStageLength * Math.sin(shoulderOffset))
                    .plus(new Translation2d(
                            secondStageLength * Math.cos(elbowOffset), secondStageLength * Math.sin(elbowOffset))); // A
            public static final double homeWrist = 0.0;
            public static final Translation2d pickup = new Translation2d(
                            baseStageLength * Math.cos(shoulderOffset), baseStageLength * Math.sin(shoulderOffset))
                    .plus(new Translation2d(
                            secondStageLength * Math.cos(elbowOffset - Units.degreesToRadians(11.5)),
                            secondStageLength * Math.sin(elbowOffset - Units.degreesToRadians(11.5)))); // B
            public static final double pickupWrist = 3.6;
            public static final Translation2d amp = new Translation2d(-1, 19); // X
            public static final double ampWrist = pickupWrist;
            public static final Translation2d dropoff = new Translation2d(0, 25); // Y
            public static final double dropoffWrist = -1.9;
            public static final double dropoffMultiplier = 12.0;
        }
    }

    public static final double fourBarOut = 20.0;
    public static final double fourBarHome = 0.75;
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
    public static final double wristPulseDist = 8192.0 * 2.0 * Math.PI;
    public static final double wristOffset = 0.0;

    public static final double indexDistance = 1000;

    public static final Mode currentMode = Mode.REAL;
    public static final RobotIdentity compRobot = RobotIdentity.COMPETITION_ROBOT_2024;

    public static final double gyroP = 0.018;
    public static final double gyroI = 0.11;
    public static final double gyroD = 0.0013;
    public static final double IZone = 10;

    public static final int pigeonID = 25;
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

    public static class VisionConstants {

        public static boolean USE_VISION = true;

        public static boolean USE_FRONT_CAMERA = true;
        public static boolean USE_BACK_LEFT_CAMERA = false;
        public static boolean USE_BACK_RIGHT_CAMERA = false;

        public static final String FRONT_CAMERA_NAME = "FrontCamera"; // LEFT
        public static final String BACK_LEFT_CAMERA_NAME = "BlackCamera"; // RIGHT
        public static final String BACK_RIGHT_CAMERA_NAME = "TEMP_NAME_CHANGE_THIS";
        public static final String NOTE_CAMERA_NAME = "NoteCamera";

        /**
         * Physical location of the left camera on the robot, relative to the center of the robot.
         */
        public static final Transform3d ROBOT_TO_FRONT_CAMERA = new Transform3d(
                new Translation3d(Units.inchesToMeters(-12.6), Units.inchesToMeters(0), Units.inchesToMeters(9)),
                new Rotation3d(0, Math.toRadians(19.565), Math.toRadians(180)));

        // public static final Transform3d ROBOT_TO_FRONT_CAMERA = new Transform3d(
        //         new Translation3d(Units.inchesToMeters(12.6), Units.inchesToMeters(0), Units.inchesToMeters(9)),
        //         new Rotation3d(180, -Math.toRadians(19.565), Math.toRadians(180)));

        /**
         * Physical location of the back camera on the robot, relative to the center of the robot.
         */
        public static final Transform3d ROBOT_TO_BACK_LEFT_CAMERA = new Transform3d(
                new Translation3d(Units.inchesToMeters(-15), Units.inchesToMeters(-0.25), Units.inchesToMeters(6)),
                new Rotation3d(0, -Math.toRadians(35), Math.toRadians(0)));

        // Physical location of the back right camera on the robot, relative to the center of the robot. CHANGE THIS

        public static final Transform3d ROBOT_TO_BACK_RIGHT_CAMERA = new Transform3d(
                new Translation3d(-Units.inchesToMeters(15.5), Units.inchesToMeters(0), Units.inchesToMeters(6.5)),
                new Rotation3d(0, Math.toRadians(39), Math.toRadians(180)));

        /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
        public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;

        public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
        public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
        public static final double NOISY_DISTANCE_METERS = 2.5;
        public static final double DISTANCE_WEIGHT = 7;
        public static final int TAG_PRESENCE_WEIGHT = 10;

        // PLACEHOLDER
        public static final double NOTE_CAMERA_HEIGHT_METERS = Units.inchesToMeters(20);
        public static final double NOTE_HEIGHT_METERS = Units.inchesToMeters(2);
        public static final double NOTE_CAMERA_PITCH_RADIANS = Units.degreesToRadians(30);

        /**
         * Standard deviations of model states. Increase these numbers to trust your
         * model's state estimates less. This
         * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
         * meters.
         */
        public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = Matrix.mat(Nat.N3(), Nat.N1())
                .fill(
                        // if these numbers are less than one, multiplying will do bad things
                        1, // x
                        1, // y
                        1 * Math.PI // theta
                        );

        /**
         * Standard deviations of the vision measurements. Increase these numbers to
         * trust global measurements from vision
         * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
         * radians.
         */
        public static final Matrix<N3, N1> STATE_STANDARD_DEVIATIONS = Matrix.mat(Nat.N3(), Nat.N1())
                .fill(
                        // if these numbers are less than one, multiplying will do bad things
                        .1, // x
                        .1, // y
                        .1);

        // Pose on the opposite side of the field. Use with `relativeTo` to flip a pose to the opposite alliance
        public static final Pose2d FLIPPING_POSE = new Pose2d(
                new Translation2d(FieldConstants.fieldLength, FieldConstants.fieldWidth), new Rotation2d(Math.PI));

        // Vision Drive Constants

        public static final double TRANSLATION_TOLERANCE_X = 0.1; // Changed from 0.05 3/26/23
        public static final double TRANSLATION_TOLERANCE_Y = 0.1; // Changed from 0.05 3/26/23
        public static final double ROTATION_TOLERANCE = 0.035;

        public static final double MAX_VELOCITY = 2; // 3
        public static final double MAX_ACCELARATION = 1; // 2
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

    public static class speakerConstants {
        public static final double speakerBlueY = Units.inchesToMeters(218.42);
        public static final double speakerRedY = Units.inchesToMeters(218.42);
        public static final double speakerBlueX = Units.inchesToMeters(0);
        public static final double speakerRedX = Units.inchesToMeters(652.73);
        public static final double speakerHeight = Units.inchesToMeters(80.515); // (82.90 + 78.13) / 2

        public static final Pose2d speakerLocBlue = new Pose2d(speakerBlueX, speakerBlueY, new Rotation2d(0));
        public static final Pose2d speakerLocRed = new Pose2d(speakerRedX, speakerRedY, new Rotation2d(Math.PI));
    }

    public static class ShooterConstants {
        public static double FOURBAR_ANGLE_THRESHOLD = 5; // Not sure what is this
        public static Measure<Distance> MAXIMUM_READYSHOOT_DISTANCE = Meters.of(Units.feetToMeters(15));

        public static double SHOOTER_SPEED = 10;

        public static InterpolatingDoubleTreeMap fourBarMap = new InterpolatingDoubleTreeMap();

        static {
            // Key: Distance
            // Value: Shooter Position
            fourBarMap.put(26.0, 8.0);
            fourBarMap.put(21.0 + (7.0 / 12.0), 8.0);
            fourBarMap.put(20.0, 8.0);
            fourBarMap.put(18.0, 6.0);
            fourBarMap.put(16.0 + (2.0 / 12.0), 2.7);
            fourBarMap.put(14.0 + (1.0 / 12.0), 1.9);
            fourBarMap.put(11.0 + (9.0 / 12.0), 1.3);
            fourBarMap.put(8.0, 0.6);
            fourBarMap.put(7.9, 18.0);
            fourBarMap.put(5.9, 20.0);
            fourBarMap.put(3.0, 20.0);
        }

        public static InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();

        static {
            // Key: Distance
            // Value: Shooter Position
            shooterMap.put(26.0, 5400.0);
            shooterMap.put(21.0 + (7.0 / 12.0), 5400.0);
            shooterMap.put(20.0, 5400.0);
            shooterMap.put(18.0, 5400.0);
            shooterMap.put(16 + (2.0 / 12.0), 5300.0);
            shooterMap.put(14.0 + (1.0 / 12.0), 5000.0);
            shooterMap.put(11.0 + (9.0 / 12.0), 4800.0);
            shooterMap.put(9.0 + (5.0 / 12.0), 4700.0);
            shooterMap.put(3.0, 4500.0);
        }
    }
}
