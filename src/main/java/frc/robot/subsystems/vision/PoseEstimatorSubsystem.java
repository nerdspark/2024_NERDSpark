package frc.robot.subsystems.vision;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
import static frc.robot.subsystems.drive.DriveConstants.thetaStdDevCoefficient;
import static frc.robot.subsystems.drive.DriveConstants.xyStdDevCoefficient;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PoseEstimatorSubsystem extends SubsystemBase {

    private final CommandSwerveDrivetrain driveTrain;

    private final Field2d field2d = new Field2d();
    private static PhotonVisionRunnable frontEstimator;
    private static PhotonVisionRunnable backLeftEstimator;
    private static PhotonVisionRunnable backRightEstimator;
    private static Notifier allNotifier;

    static {
    }
    // private final Notifier backNotifier = new Notifier(backEstimator);

    private OriginPosition originPosition = kBlueAllianceWallRightSide;

    public PoseEstimatorSubsystem(CommandSwerveDrivetrain driveTrain) {

        this.driveTrain = driveTrain;
        if (VisionConstants.USE_VISION == true) {
            if (VisionConstants.USE_FRONT_CAMERA) {
                frontEstimator = new PhotonVisionRunnable(
                        VisionConstants.FRONT_CAMERA_NAME, VisionConstants.ROBOT_TO_FRONT_CAMERA);
            }
            if (VisionConstants.USE_BACK_LEFT_CAMERA) {
                backLeftEstimator = new PhotonVisionRunnable(
                        VisionConstants.BACK_LEFT_CAMERA_NAME,
                        VisionConstants.ROBOT_TO_BACK_LEFT_CAMERA);
            }
            if (VisionConstants.USE_BACK_RIGHT_CAMERA) {
                backRightEstimator = new PhotonVisionRunnable(
                        VisionConstants.BACK_RIGHT_CAMERA_NAME,
                        VisionConstants.ROBOT_TO_BACK_RIGHT_CAMERA);
            }

            allNotifier = new Notifier(() -> {
                if (VisionConstants.USE_FRONT_CAMERA) {
                    frontEstimator.run();
                }
                if (VisionConstants.USE_BACK_LEFT_CAMERA) {
                    backLeftEstimator.run();
                }

                if (VisionConstants.USE_BACK_RIGHT_CAMERA) {
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

    @Override
    public void periodic() {
        // Update pose estimator with drivetrain sensors

        if (VisionConstants.USE_VISION == true) {
            if (VisionConstants.USE_FRONT_CAMERA) {
                updatePoseEstimates(frontEstimator);
            }
            if (VisionConstants.USE_BACK_LEFT_CAMERA) {
                updatePoseEstimates(backLeftEstimator);
            }
            if (VisionConstants.USE_BACK_RIGHT_CAMERA) {
                updatePoseEstimates(backRightEstimator);
            }
        } else {
            if (allNotifier != null) allNotifier.close();
        }

        // Set the pose on the dashboard
        var dashboardPose = getCurrentPose();
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
            // var poseStrategyUsed = cameraPose.strategy;

            int[] tagIDsFrontCamera = new int[cameraPose.targetsUsed.size()];
            double averageTagDistance = 0.0;
            // double poseAmbiguity = 0.0;
            double smallestDistance = Double.POSITIVE_INFINITY;

            for (int i = 0; i < cameraPose.targetsUsed.size(); i++) {
                tagIDsFrontCamera[i] =
                        (int) cameraPose.targetsUsed.get(i).getFiducialId(); // Retrieves and stores the tag ID
                if (VisionConstants.VISION_DEV_DIST_STRATEGY
                        == VisionConstants.VisionDeviationDistanceStrategy.AVERAGE_DISTANCE) {

                    averageTagDistance += cameraPose
                            .targetsUsed
                            .get(i)
                            .getBestCameraToTarget()
                            .getTranslation()
                            .getNorm(); // Calculates the sum of the tag distances

                } else {
                    var distance = cameraPose
                            .targetsUsed
                            .get(i)
                            .getBestCameraToTarget()
                            .getTranslation()
                            .getNorm(); // Calculates the distance to the tag

                    if (distance < smallestDistance) smallestDistance = distance;
                }
                var distanceUsedForCalculatingStdDev = 0.0;
                if (VisionConstants.VISION_DEV_DIST_STRATEGY
                        == VisionConstants.VisionDeviationDistanceStrategy.AVERAGE_DISTANCE) {
                    averageTagDistance /= cameraPose.targetsUsed.size(); // Calculates the average tag distance
                    distanceUsedForCalculatingStdDev = averageTagDistance;
                } else {
                    distanceUsedForCalculatingStdDev = smallestDistance;
                }
                // if ((poseStrategyUsed != PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR))
                //     poseAmbiguity /= cameraPose.targetsUsed.size(); // Calculates the average tag pose ambiguity

                double timestamp = cameraPose.timestampSeconds;
                Pose3d robotPose = cameraPose.estimatedPose;
                // Correct the pose for rotation as camera is mounted on the back side
                // robotPose = robotPose.plus(new Transform3d(new Translation3d(), new Rotation3d(0, 0, Math.PI)));
                robotPose = robotPose.plus(VisionConstants.ROBOT_TO_FRONT_CAMERA);

                double xyStdDev = calculateXYStdDev(distanceUsedForCalculatingStdDev, cameraPose.targetsUsed.size());
                double thetaStdDev =
                        calculateThetaStdDev(distanceUsedForCalculatingStdDev, cameraPose.targetsUsed.size());
                driveTrain.addVisionMeasurement(
                        robotPose.toPose2d(), timestamp, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
            }
        }
    }
}
