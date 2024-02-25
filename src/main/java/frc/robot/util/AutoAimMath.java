package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.speakerConstants;
import java.util.function.Supplier;

public class AutoAimMath {
    Supplier<Pose2d> poseSupplier;
    Pose2d targetPose;

    public static Rotation2d getAutoAimCalcRobot(Supplier<Pose2d> poseSupplier, Translation2d targetPose) {

        double robotAimAngle = Math.atan2(
                        targetPose.getY() - poseSupplier.get().getY(),
                        AllianceFlipUtil.apply(targetPose.getX())
                                - poseSupplier.get().getX())
                - 180; // Robot Angle

        return new Rotation2d(robotAimAngle);
    }

    public static double getAutoAimCalcFourBar(Supplier<Pose2d> poseSupplier, Pose2d targetPose) {

        double fourBarAngle = Math.atan2(
                speakerConstants.speakerHeight,
                poseSupplier
                        .get()
                        .getTranslation()
                        .getDistance(AllianceFlipUtil.apply(targetPose.getTranslation()))); // Shooter Angle

        return fourBarAngle;
    }
}
