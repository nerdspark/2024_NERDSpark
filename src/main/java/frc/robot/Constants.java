package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static class VisionConstants {

        public static boolean USE_VISION = true;

        public static final String LEFT_CAMERA_NAME = "BlueCamera";
        public static final String RIGHT_CAMERA_NAME = "BlackCamera";

        /**
         * Physical location of the left camera on the robot, relative to the center of the robot.
         */
        public static final Transform3d ROBOT_TO_LEFT_CAMERA = new Transform3d(
                new Translation3d(Units.inchesToMeters(15.5), Units.inchesToMeters(6.5), Units.inchesToMeters(2.5)),
                new Rotation3d(0, Math.toRadians(0), Math.toRadians(0)));

        /**
         * Physical location of the back camera on the robot, relative to the center of the robot.
         */
        public static final Transform3d ROBOT_TO_RIGHT_CAMERA = new Transform3d(
                new Translation3d(Units.inchesToMeters(15.5), Units.inchesToMeters(-7), Units.inchesToMeters(2.5)),
                new Rotation3d(0, Math.toRadians(45), Math.toRadians(0)));

        /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
        public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;

        public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
        public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
        public static final double NOISY_DISTANCE_METERS = 2.5;
        public static final double DISTANCE_WEIGHT = 7;
        public static final int TAG_PRESENCE_WEIGHT = 10;

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

        public static final double FIELD_LENGTH_METERS = 16.54175;

        public static final double FIELD_WIDTH_METERS = 8.0137;

        // Pose on the opposite side of the field. Use with `relativeTo` to flip a pose to the opposite alliance
        public static final Pose2d FLIPPING_POSE =
                new Pose2d(new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS), new Rotation2d(Math.PI));

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

    public static double xOffset = 50;
    public static double yOffset = 30;
    public static final Pose2d[] targetPoses = new Pose2d[] {
        new Pose2d(Units.inchesToMeters(610.77 - xOffset), Units.inchesToMeters(42.19 - yOffset), new Rotation2d(0)),
        new Pose2d(Units.inchesToMeters(610.77 - xOffset), Units.inchesToMeters(42.19), new Rotation2d(0)),
        new Pose2d(Units.inchesToMeters(610.77 - xOffset), Units.inchesToMeters(42.19 + yOffset), new Rotation2d(0)),
        new Pose2d(Units.inchesToMeters(610.77 - xOffset), Units.inchesToMeters((108.19 - yOffset)), new Rotation2d(0)),
        new Pose2d(Units.inchesToMeters(610.77 - xOffset), Units.inchesToMeters((108.19)), new Rotation2d(0)),
        new Pose2d(Units.inchesToMeters(610.77 - xOffset), Units.inchesToMeters((108.19 + yOffset)), new Rotation2d(0)),
        new Pose2d(Units.inchesToMeters(610.77 - xOffset), Units.inchesToMeters(174.19 - yOffset), new Rotation2d(0)),
        new Pose2d(Units.inchesToMeters(610.77 - xOffset), Units.inchesToMeters(174.19), new Rotation2d(0)),
        new Pose2d(Units.inchesToMeters(610.77 - xOffset), Units.inchesToMeters(174.19 + yOffset), new Rotation2d(0))
    };

    public static final Pose2d[] targetPosesBlue = new Pose2d[] {
        new Pose2d(
                Units.inchesToMeters(40.45 + xOffset), Units.inchesToMeters(42.19 - yOffset), new Rotation2d(Math.PI)),
        new Pose2d(Units.inchesToMeters(40.45 + xOffset), Units.inchesToMeters(42.19), new Rotation2d(Math.PI)),
        new Pose2d(
                Units.inchesToMeters(40.45 + xOffset), Units.inchesToMeters(42.19 + yOffset), new Rotation2d(Math.PI)),
        new Pose2d(
                Units.inchesToMeters(40.45 + xOffset), Units.inchesToMeters(108.19 - yOffset), new Rotation2d(Math.PI)),
        new Pose2d(Units.inchesToMeters(40.45 + xOffset), Units.inchesToMeters((108.19)), new Rotation2d(Math.PI)),
        new Pose2d(
                Units.inchesToMeters(40.45 + xOffset),
                Units.inchesToMeters((108.19 + yOffset)),
                new Rotation2d(Math.PI)),
        new Pose2d(
                Units.inchesToMeters(40.45 + xOffset), Units.inchesToMeters(174.19 - yOffset), new Rotation2d(Math.PI)),
        new Pose2d(Units.inchesToMeters(40.45 + xOffset), Units.inchesToMeters(174.19), new Rotation2d(Math.PI)),
        new Pose2d(
                Units.inchesToMeters(40.45 + xOffset), Units.inchesToMeters(174.19 + yOffset), new Rotation2d(Math.PI))
    };
}
