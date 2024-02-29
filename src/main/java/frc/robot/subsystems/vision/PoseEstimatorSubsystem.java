package frc.robot.subsystems.vision;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;
import static frc.robot.subsystems.drive.DriveConstants.thetaStdDevCoefficient;
import static frc.robot.subsystems.drive.DriveConstants.xyStdDevCoefficient;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.FieldConstants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PoseEstimatorSubsystem extends SubsystemBase {

    private final CommandSwerveDrivetrain driveTrain;

    // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
    // you trust your various sensors. Smaller numbers will cause the filter to
    // "trust" the estimate from that particular component more than the others.
    // This in turn means the particualr component will have a stronger influence
    // on the final pose estimate.

    // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
    // you trust your various sensors. Smaller numbers will cause the filter to
    // "trust" the estimate from that particular component more than the others.
    // This in turn means the particualr component will have a stronger influence
    // on the final pose estimate.

    private final Field2d field2d = new Field2d();
    private static PhotonVisionRunnable frontEstimator;
    private static PhotonVisionRunnable backLeftEstimator;
    private static PhotonVisionRunnable backRightEstimator;
    private static Notifier allNotifier;

    static {
    }
    // private final Notifier backNotifier = new Notifier(backEstimator);

    private OriginPosition originPosition = kBlueAllianceWallRightSide;

    // private final ArrayList<Double> xValues = new ArrayList<Double>();
    // private final ArrayList<Double> yValues = new ArrayList<Double>();

    public PoseEstimatorSubsystem(CommandSwerveDrivetrain driveTrain) {

        this.driveTrain = driveTrain;
        if (Constants.VisionConstants.USE_VISION == true) {
            if (Constants.VisionConstants.USE_FRONT_CAMERA) {
                frontEstimator = new PhotonVisionRunnable(
                        Constants.VisionConstants.FRONT_CAMERA_NAME, Constants.VisionConstants.ROBOT_TO_FRONT_CAMERA);
            }
            if (Constants.VisionConstants.USE_BACK_LEFT_CAMERA) {
                backLeftEstimator = new PhotonVisionRunnable(
                        Constants.VisionConstants.BACK_LEFT_CAMERA_NAME,
                        Constants.VisionConstants.ROBOT_TO_BACK_LEFT_CAMERA);
            }
            if (Constants.VisionConstants.USE_BACK_RIGHT_CAMERA) {
                backRightEstimator = new PhotonVisionRunnable(
                        Constants.VisionConstants.BACK_RIGHT_CAMERA_NAME,
                        Constants.VisionConstants.ROBOT_TO_BACK_RIGHT_CAMERA);
            }

            allNotifier = new Notifier(() -> {
                if (Constants.VisionConstants.USE_FRONT_CAMERA) {
                    frontEstimator.run();
                }
                if (Constants.VisionConstants.USE_BACK_LEFT_CAMERA) {
                    backLeftEstimator.run();
                }

                if (Constants.VisionConstants.USE_BACK_RIGHT_CAMERA) {
                    backRightEstimator.run();
                }
            });

            allNotifier.setName("runAll");
            allNotifier.startPeriodic(0.02);
        }
    }

    public void addDashboardWidgets(ShuffleboardTab tab) {
        tab.add("Field", field2d).withPosition(0, 0).withSize(6, 4);
        tab.addString("Pose", this::getFomattedPose).withPosition(6, 2).withSize(2, 1);
    }

    /**
     * Sets the alliance. This is used to configure the origin of the AprilTag map
     *
     * @param alliance alliance
     */
    public void setAlliance(Alliance alliance) {
        boolean allianceChanged = false;
        switch (alliance) {
            case Blue:
                allianceChanged = (originPosition == kRedAllianceWallRightSide);
                originPosition = kBlueAllianceWallRightSide;
                break;
            case Red:
                allianceChanged = (originPosition == kBlueAllianceWallRightSide);
                originPosition = kRedAllianceWallRightSide;
                break;
            default:
                // No valid alliance data. Nothing we can do about it
        }

        if (allianceChanged) {
            // The alliance changed, which changes the coordinate system.
            // Since a tag was seen, and the tags are all relative to the coordinate system,
            // the estimated pose
            // needs to be transformed to the new coordinate system.
            var newPose = flipAlliance(getCurrentPose());
            driveTrain.seedFieldRelative(newPose);
        }
    }

    @Override
    public void periodic() {
        // Update pose estimator with drivetrain sensors

        if (Constants.VisionConstants.USE_VISION == true) {
            if (Constants.VisionConstants.USE_FRONT_CAMERA) {
                updatePoseEstimates(frontEstimator);
            }
            if (Constants.VisionConstants.USE_BACK_LEFT_CAMERA) {
                updatePoseEstimates(backLeftEstimator);
            }
            if (Constants.VisionConstants.USE_BACK_RIGHT_CAMERA) {
                updatePoseEstimates(backRightEstimator);
            }
        } else {
            if (allNotifier != null) allNotifier.close();
        }

        // estimatorChecker(backEstimator);

        // Set the pose on the dashboard
        var dashboardPose = getCurrentPose();
        if (originPosition == kRedAllianceWallRightSide) {
            // Flip the pose when red, since the dashboard field photo cannot be rotated
            // dashboardPose = flipAlliance(dashboardPose);
            dashboardPose = dashboardPose.rotateBy(new Rotation2d(Units.degreesToRadians(180)));
        }
        field2d.setRobotPose(dashboardPose);
    }

    private String getFomattedPose() {
        var pose = getCurrentPose();
        return String.format(
                "(%.3f, %.3f) %.2f degrees",
                pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }

    public Pose2d getCurrentPose() {
        return driveTrain.getState().Pose;
    }

    /**
     * Resets the current pose to the specified pose. This should ONLY be called
     * when the robot's position on the field is known, like at the beginning of
     * a match.
     *
     * @param newPose new pose
     */
    public void setCurrentPose(Pose2d newPose) {
        driveTrain.seedFieldRelative(newPose);
    }

    /**
     * Resets the position on the field to 0,0 0-degrees, with forward being
     * downfield. This resets
     * what "forward" is for field oriented driving.
     */
    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }

    /**
     * Transforms a pose to the opposite alliance's coordinate system. (0,0) is
     * always on the right corner of your
     * alliance wall, so for 2023, the field elements are at different coordinates
     * for each alliance.
     *
     * @param poseToFlip pose to transform to the other alliance
     * @return pose relative to the other alliance's coordinate system
     */
    private Pose2d flipAlliance(Pose2d poseToFlip) {
        return poseToFlip.relativeTo(new Pose2d(
                new Translation2d(FieldConstants.fieldLength, FieldConstants.fieldWidth), new Rotation2d(Math.PI)));
    }

    // public void resetPoseRating() {
    // xValues.clear();
    // yValues.clear();
    // }

    private Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation) {
        double smallestDistance = Double.POSITIVE_INFINITY;
        for (var target : estimation.targetsUsed) {
            var t3d = target.getBestCameraToTarget();
            var distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
            if (distance < smallestDistance) smallestDistance = distance;
        }
        double poseAmbiguityFactor = estimation.targetsUsed.size() != 1
                ? 1
                : Math.max(
                        1,
                        (estimation.targetsUsed.get(0).getPoseAmbiguity()
                                        + Constants.VisionConstants.POSE_AMBIGUITY_SHIFTER)
                                * Constants.VisionConstants.POSE_AMBIGUITY_MULTIPLIER);
        double confidenceMultiplier = Math.max(
                1,
                (Math.max(
                                        1,
                                        Math.max(0, smallestDistance - Constants.VisionConstants.NOISY_DISTANCE_METERS)
                                                * Constants.VisionConstants.DISTANCE_WEIGHT)
                                * poseAmbiguityFactor)
                        / (1 + ((estimation.targetsUsed.size() - 1) * Constants.VisionConstants.TAG_PRESENCE_WEIGHT)));

        return Constants.VisionConstants.VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier);
    }

    public void estimatorChecker(PhotonVisionRunnable estamator) {
        var cameraPose = estamator.grabLatestEstimatedPose();
        if (cameraPose != null) {
            // New pose from vision
            var pose2d = cameraPose.estimatedPose.toPose2d();
            if (originPosition == kRedAllianceWallRightSide) {
                pose2d = flipAlliance(pose2d);
            }
            driveTrain.addVisionMeasurement(pose2d, cameraPose.timestampSeconds, confidenceCalculator(cameraPose));
            // confidenceCalculator(cameraPose);
        }
    }

    /**
     * Calculate the standard deviation of the x and y coordinates.
     *
     * @param poseEstimates The pose estimate
     * @param tagPosesSize The number of detected tag poses
     * @return The standard deviation of the x and y coordinates
     */
    private double calculateXYStdDev(Double avgTagDistance, int tagPosesSize) {
        return xyStdDevCoefficient * Math.pow(avgTagDistance, 2.0) / tagPosesSize;
    }
    /**
     * Calculate the standard deviation of the theta coordinate.
     *
     * @param poseEstimates The pose estimate
     * @param tagPosesSize The number of detected tag poses
     * @return The standard deviation of the theta coordinate
     */
    private double calculateThetaStdDev(Double avgTagDistance, int tagPosesSize) {
        return thetaStdDevCoefficient * Math.pow(avgTagDistance, 2.0) / tagPosesSize;
    }

    /**
     * Updates the inputs for AprilTag vision.
     *
     * @param estimator PhotonVisionRunnable estimator.
     * @param inputs The AprilTagVisionIOInputs object containing the inputs.
     */
    public void updatePoseEstimates(PhotonVisionRunnable estomator) {

        var cameraPose = estomator.grabLatestEstimatedPose();

        if (cameraPose != null) {
            var poseStrategyUsed = cameraPose.strategy;

            int[] tagIDsFrontCamera = new int[cameraPose.targetsUsed.size()];
            double averageTagDistance = 0.0;
            double poseAmbiguity = 0.0;

            for (int i = 0; i < cameraPose.targetsUsed.size(); i++) {
                tagIDsFrontCamera[i] =
                        (int) cameraPose.targetsUsed.get(i).getFiducialId(); // Retrieves and stores the tag ID
                averageTagDistance += cameraPose
                        .targetsUsed
                        .get(i)
                        .getBestCameraToTarget()
                        .getTranslation()
                        .getNorm(); // Calculates the sum of the tag distances

                if ((poseStrategyUsed != PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR))
                    poseAmbiguity += cameraPose.targetsUsed.get(i).getPoseAmbiguity();
            }

            averageTagDistance /= cameraPose.targetsUsed.size(); // Calculates the average tag distance

            if ((poseStrategyUsed != PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR))
                poseAmbiguity /= cameraPose.targetsUsed.size(); // Calculates the average tag pose ambiguity

            double timestamp = cameraPose.timestampSeconds;
            Pose3d robotPose = cameraPose.estimatedPose;

            double xyStdDev = calculateXYStdDev(averageTagDistance, cameraPose.targetsUsed.size());
            double thetaStdDev = calculateThetaStdDev(averageTagDistance, cameraPose.targetsUsed.size());
            driveTrain.addVisionMeasurement(
                    robotPose.toPose2d(), timestamp, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
        }
    }
}
