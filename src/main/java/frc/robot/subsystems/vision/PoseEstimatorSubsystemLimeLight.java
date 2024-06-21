package frc.robot.subsystems.vision;

import static frc.robot.subsystems.drive.DriveConstants.thetaStdDevCoefficient;
import static frc.robot.subsystems.drive.DriveConstants.xyStdDevCoefficient;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.LimelightHelpers;

public class PoseEstimatorSubsystemLimeLight extends SubsystemBase {

    private final CommandSwerveDrivetrain driveTrain;

    private final Field2d field2d = new Field2d();

    static {
    }
    // private final Notifier backNotifier = new Notifier(backEstimator);

    public PoseEstimatorSubsystemLimeLight(CommandSwerveDrivetrain driveTrain) {

        this.driveTrain = driveTrain;

        LimelightHelpers.setPipelineIndex(VisionConstants.Limelight1, 0);
        LimelightHelpers.setPipelineIndex(VisionConstants.Limelight2, 0);
        LimelightHelpers.setPipelineIndex(VisionConstants.Limelight3, 0);
    }

    public void addDashboardWidgets(ShuffleboardTab tab) {
        tab.add("Field", field2d).withPosition(0, 0).withSize(6, 4);
        tab.addString("Pose", this::getFomattedPose).withPosition(6, 2).withSize(2, 1);
    }

    @Override
    public void periodic() {
        // Update pose estimator with drivetrain sensors

        if (VisionConstants.USE_VISION == true) {
            if (VisionConstants.USE_FRONT_LIMELIGHT) {
                updatePoseEstimates(VisionConstants.Limelight1);
            }
            if (VisionConstants.USE_BACK_LEFT_LIMELIGHT) {
                updatePoseEstimates(VisionConstants.Limelight2);
            }
            if (VisionConstants.USE_BACK_RIGHT_LIMELIGHT) {
                updatePoseEstimates(VisionConstants.Limelight3);
            }
        }

        // Set the pose on the dashboard. Not needed. Done in drive train
        // var dashboardPose = getCurrentPose();
        // field2d.setRobotPose(dashboardPose);
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
    public void updatePoseEstimates(String limelightName) {
        // LimelightHelpers.SetRobotOrientation(limelightName, driveTrain.getPigeon2().getYaw().getValue(), 0, 0, 0, 0,
        // 0);

        LimelightHelpers.PoseEstimate cameraPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        // Uncomment SetRobotOrientation line if trying MegaTag2 below.
        // LimelightHelpers.PoseEstimate cameraPose =
        // LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        var distanceUsedForCalculatingStdDev = cameraPose.avgTagDist;
        double timestamp = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName).timestampSeconds;
        double xyStdDev = calculateXYStdDev(distanceUsedForCalculatingStdDev, cameraPose.tagCount);
        double thetaStdDev = calculateThetaStdDev(distanceUsedForCalculatingStdDev, cameraPose.tagCount);
        if (cameraPose.tagCount != 0 && Math.abs(driveTrain.getPigeon2().getRate()) < 720) {
            driveTrain.addVisionMeasurement(
                    cameraPose.pose, cameraPose.timestampSeconds, VecBuilder.fill(xyStdDev, xyStdDev, 1));
        }
    }
}
