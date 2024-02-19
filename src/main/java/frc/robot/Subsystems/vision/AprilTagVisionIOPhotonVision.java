// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Constants;
import frc.robot.subsystems.PhotonVisionRunnable;
import frc.robot.util.VisionHelpers;
import frc.robot.util.VisionHelpers.PoseEstimate;
import java.util.ArrayList;

/** This class represents the implementation of AprilTagVisionIO using Limelight camera. */
public class AprilTagVisionIOPhotonVision implements AprilTagVisionIO {

    private static PhotonVisionRunnable frontEstimator;
    private static PhotonVisionRunnable backEstimator;
    private static Notifier allNotifier;

    private OriginPosition originPosition = kBlueAllianceWallRightSide;

    /**
     * Constructs a new AprilTagVisionIOLimelight instance.
     *
     */
    public AprilTagVisionIOPhotonVision() {
        if (Constants.VisionConstants.USE_VISION == true) {
            backEstimator = new PhotonVisionRunnable(
                    Constants.VisionConstants.BACK_CAMERA_NAME, Constants.VisionConstants.ROBOT_TO_BACK_CAMERA);
            frontEstimator = new PhotonVisionRunnable(
                    Constants.VisionConstants.FRONT_CAMERA_NAME, Constants.VisionConstants.ROBOT_TO_FRONT_CAMERA);

            allNotifier = new Notifier(() -> {
                frontEstimator.run();
                backEstimator.run();
            });

            allNotifier.setName("runAll");
            allNotifier.startPeriodic(0.02);
        }
    }

    /**
     * Updates the inputs for AprilTag vision.
     *
     * @param inputs The AprilTagVisionIOInputs object containing the inputs.
     */
    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {

        if (Constants.VisionConstants.USE_VISION == true) {
            updatePoseEstimates(frontEstimator, inputs);
            updatePoseEstimates(backEstimator, inputs);
        }
    }

    /**
     * Updates the inputs for AprilTag vision.
     *
     * @param estimator PhotonVisionRunnable estimator.
     * @param inputs The AprilTagVisionIOInputs object containing the inputs.
     */
    public void updatePoseEstimates(PhotonVisionRunnable estomator, AprilTagVisionIOInputs inputs) {

        var cameraPose = estomator.grabLatestEstimatedPose();

        ArrayList<PoseEstimate> poseEstimates = new ArrayList<>(); // Creates an empty ArrayList to store pose estimates

        if (cameraPose != null) {
            // New pose from vision
            var cameraPose2d = cameraPose.estimatedPose.toPose2d();
            if (originPosition == kRedAllianceWallRightSide) {
                cameraPose2d = VisionHelpers.flipAlliance(cameraPose2d);
            }

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

                poseAmbiguity += cameraPose.targetsUsed.get(i).getPoseAmbiguity();               

            }
            averageTagDistance /= cameraPose.targetsUsed.size(); // Calculates the average tag distance

            poseAmbiguity /= cameraPose.targetsUsed.size(); // Calculates the average tag pose ambiguity

            poseEstimates.add(new PoseEstimate(
                    cameraPose.estimatedPose,
                    cameraPose.timestampSeconds,
                    averageTagDistance,
                    tagIDsFrontCamera,
                    poseAmbiguity)); //

            inputs.poseEstimates = poseEstimates;
        }
    }
}
