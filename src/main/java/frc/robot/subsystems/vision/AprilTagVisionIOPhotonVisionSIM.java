// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.util.FieldConstants;
import frc.robot.util.VisionHelpers.PoseEstimate;
import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class AprilTagVisionIOPhotonVisionSIM implements AprilTagVisionIO {
    private final PhotonCamera frontCamera;
    private final PhotonCamera backLeftCamera;
    private final PhotonCamera backRightCamera;

    private final PhotonPoseEstimator photonEstimatorFront;
    private final PhotonPoseEstimator photonEstimatorBackLeft;
    private final PhotonPoseEstimator photonEstimatorBackRight;

    private VisionSystemSim visionSim;

    private PhotonCameraSim cameraSimFront;
    private PhotonCameraSim cameraSimBackLeft;
    private PhotonCameraSim cameraSimBackRight;

    private double lastEstTimestamp = 0;
    private final Supplier<Pose2d> poseSupplier;

    /**
     * Constructs a new AprilTagVisionIOPhotonVisionSIM instance.
     *
     * @param identifier The identifier of the PhotonCamera.
     * @param robotToCamera The transform from the robot's coordinate system to the camera's
     *     coordinate system.
     * @param poseSupplier The supplier of the robot's pose.
     */
    public AprilTagVisionIOPhotonVisionSIM(Supplier<Pose2d> poseSupplier) {
        this.frontCamera = new PhotonCamera(Constants.VisionConstants.FRONT_CAMERA_NAME);
        this.backLeftCamera = new PhotonCamera(Constants.VisionConstants.BACK_LEFT_CAMERA_NAME);
        this.backRightCamera = new PhotonCamera(Constants.VisionConstants.BACK_RIGHT_CAMERA_NAME);

        photonEstimatorFront = new PhotonPoseEstimator(
                FieldConstants.aprilTags,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                frontCamera,
                Constants.VisionConstants.ROBOT_TO_FRONT_CAMERA);
        photonEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        photonEstimatorBackLeft = new PhotonPoseEstimator(
                FieldConstants.aprilTags,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                backLeftCamera,
                Constants.VisionConstants.ROBOT_TO_BACK_LEFT_CAMERA);
        photonEstimatorBackLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        photonEstimatorBackRight = new PhotonPoseEstimator(
                FieldConstants.aprilTags,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                backRightCamera,
                Constants.VisionConstants.ROBOT_TO_BACK_RIGHT_CAMERA);
        photonEstimatorBackRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // Create the vision system simulation which handles cameras and targets on the
        // field.
        visionSim = new VisionSystemSim("main");
        // Add all the AprilTags inside the tag layout as visible targets to this
        // simulated field.
        visionSim.addAprilTags(FieldConstants.aprilTags);
        // Create simulated camera properties. These can be set to mimic your actual
        // camera.

        cameraSimFront = new PhotonCameraSim(frontCamera);
        cameraSimBackLeft = new PhotonCameraSim(backLeftCamera);
        cameraSimBackRight = new PhotonCameraSim(backRightCamera);

        SimCameraProperties cameraProps = getCameraProp();

        cameraSimFront = new PhotonCameraSim(frontCamera, cameraProps);
        cameraSimBackLeft = new PhotonCameraSim(backLeftCamera, cameraProps);
        cameraSimBackRight = new PhotonCameraSim(backRightCamera, cameraProps);

        // Add the simulated camera to view the targets on this simulated field.
        visionSim.addCamera(cameraSimFront, Constants.VisionConstants.ROBOT_TO_FRONT_CAMERA);
        visionSim.addCamera(cameraSimBackLeft, Constants.VisionConstants.ROBOT_TO_BACK_LEFT_CAMERA);
        visionSim.addCamera(cameraSimBackRight, Constants.VisionConstants.ROBOT_TO_BACK_RIGHT_CAMERA);

        cameraSimFront.enableDrawWireframe(true);
        cameraSimBackLeft.enableDrawWireframe(true);
        cameraSimBackRight.enableDrawWireframe(true);

        this.poseSupplier = poseSupplier;
    }

    public SimCameraProperties getCameraProp() {

        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        cameraProp.setCalibError(0.35, 0.10);
        cameraProp.setFPS(15);
        cameraProp.setAvgLatencyMs(50);
        cameraProp.setLatencyStdDevMs(15);

        return cameraProp;
    }
    /**
     * Updates the inputs for AprilTag vision.
     *
     * @param inputs The AprilTagVisionIOInputs object containing the inputs.
     */
    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {

        updatePoseEstimates(inputs, cameraSimFront, photonEstimatorFront);
        updatePoseEstimates(inputs, cameraSimBackLeft, photonEstimatorBackLeft);
        updatePoseEstimates(inputs, cameraSimBackRight, photonEstimatorBackRight);
    }

    /** Updates the PhotonPoseEstimator and returns the estimated global pose. */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(
            PhotonCamera camera, PhotonPoseEstimator photonEstimator) {
        var visionEst = photonEstimator.update();
        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        visionEst.ifPresentOrElse(
                est -> getSimDebugField().getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d()), () -> {
                    if (newResult)
                        getSimDebugField().getObject("VisionEstimation").setPoses();
                });
        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst;
    }

    public void updatePoseEstimates(
            AprilTagVisionIOInputs inputs, PhotonCameraSim cameraSim, PhotonPoseEstimator photonEstimator) {

        visionSim.update(poseSupplier.get());
        PhotonPipelineResult results = cameraSim.getCamera().getLatestResult();
        ArrayList<PoseEstimate> poseEstimates = new ArrayList<>();
        double timestamp = results.getTimestampSeconds();
        Optional<Alliance> allianceOptional = DriverStation.getAlliance();
        if (!results.targets.isEmpty() && allianceOptional.isPresent()) {
            double latencyMS = results.getLatencyMillis();
            Pose3d poseEstimation;
            Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose(cameraSim.getCamera(), photonEstimator);
            if (estimatedPose.isEmpty()) {
                return;
            }
            poseEstimation = estimatedPose.get().estimatedPose;
            double averageTagDistance = 0.0;
            timestamp -= (latencyMS / 1e3);
            int[] tagIDs = new int[results.targets.size()];
            for (int i = 0; i < results.targets.size(); i++) {
                tagIDs[i] = results.targets.get(i).getFiducialId();
                var tagPose = photonEstimator.getFieldTags().getTagPose(tagIDs[i]);
                if (tagPose.isEmpty()) {
                    continue;
                }
                averageTagDistance += tagPose.get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(poseEstimation.getTranslation().toTranslation2d());
            }
            averageTagDistance /= tagIDs.length;
            poseEstimates.add(new PoseEstimate(poseEstimation, timestamp, averageTagDistance, tagIDs, 0));
            inputs.poseEstimates = poseEstimates;
        }
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!RobotBase.isSimulation()) return null;
        return visionSim.getDebugField();
    }
}
