package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import java.util.function.Supplier;

public class AutoAim {
    Supplier<Pose2d> poseSupplier;

    public AutoAim(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
    }

    public static double calculateFourBarPosition(Supplier<Pose2d> poseSupplier) {
        Pose2d speakerPose;
        if (DriverStation.getAlliance().equals(Alliance.Blue)) {
            speakerPose = Constants.speakerConstants.speakerLocBlue;
        } else {
            speakerPose = Constants.speakerConstants.speakerLocRed;
        }
        double distanceToSpeaker = poseSupplier.get().getTranslation().getDistance(speakerPose.getTranslation());
        return Constants.ShooterConstants.fourBarMap.get(Units.metersToFeet(distanceToSpeaker));
    }

    public static double calculateShooterRPM(Supplier<Pose2d> poseSupplier) {
        Pose2d speakerPose;
        if (DriverStation.getAlliance().equals(Alliance.Blue)) {
            speakerPose = Constants.speakerConstants.speakerLocBlue;
        } else {
            speakerPose = Constants.speakerConstants.speakerLocRed;
        }
        double distanceToSpeaker = poseSupplier.get().getTranslation().getDistance(speakerPose.getTranslation());
        return Constants.ShooterConstants.shooterMap.get(Units.metersToFeet(distanceToSpeaker));
    }

    public static Rotation2d calculateAngleToSpeaker(Supplier<Pose2d> poseSupplier) {
        Pose2d speakerPose;
        if (DriverStation.getAlliance().equals(Alliance.Blue)) {
            speakerPose = Constants.speakerConstants.speakerLocBlue;
        } else {
            speakerPose = Constants.speakerConstants.speakerLocRed;
        }

        // ALTERNATE LOGIC
        // return speakerPose.getTranslation().minus(poseSupplier.get().getTranslation()).getAngle();

        double robotAimAngle = Math.atan2(
                speakerPose.getY() - poseSupplier.get().getY(),
                speakerPose.getX() - poseSupplier.get().getX()); // Robot Angle

        return new Rotation2d(robotAimAngle);
    }
}
