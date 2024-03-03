package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import java.util.function.Supplier;

public class AutoAim {
    Supplier<Pose2d> poseSupplier;

    public AutoAim(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
    }

    public static double calculateFourBarPosition(Supplier<Pose2d> poseSupplier, Supplier<Translation2d> speeds) {
        Pose2d speakerPose;
        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            speakerPose = Constants.speakerConstants.speakerLocBlue;
        } else {
            speakerPose = Constants.speakerConstants.speakerLocRed;
        }
        double distanceToSpeaker = poseSupplier.get().getTranslation().getDistance(speakerPose.getTranslation());
        double distanceToSpeaker2 = poseSupplier
                .get()
                .getTranslation()
                .plus(speeds.get().times(distanceToSpeaker * Constants.shootMoveMultiplier))
                .getDistance(speakerPose.getTranslation());
        double distanceToSpeaker3 = poseSupplier
                .get()
                .getTranslation()
                .plus(speeds.get().times(distanceToSpeaker2 * Constants.shootMoveMultiplier))
                .getDistance(speakerPose.getTranslation());

        double angle = Constants.ShooterConstants.fourBarMap.get(
                Units.metersToFeet(distanceToSpeaker3 + ShooterConstants.CONSTANT_DISTANCE_ADD));
        SmartDashboard.putNumber("Calculated 4Bar", angle);
        return angle;
    }

    public static double calculateShooterRPM(Supplier<Pose2d> poseSupplier, Supplier<Translation2d> speeds) {
        Pose2d speakerPose;
        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            speakerPose = Constants.speakerConstants.speakerLocBlue;
        } else {
            speakerPose = Constants.speakerConstants.speakerLocRed;
        }
        double distanceToSpeaker = poseSupplier.get().getTranslation().getDistance(speakerPose.getTranslation());
        double distanceToSpeaker2 = poseSupplier
                .get()
                .getTranslation()
                .plus(speeds.get().times(distanceToSpeaker * Constants.shootMoveMultiplier))
                .getDistance(speakerPose.getTranslation());
        double distanceToSpeaker3 = poseSupplier
                .get()
                .getTranslation()
                .plus(speeds.get().times(distanceToSpeaker2 * Constants.shootMoveMultiplier))
                .getDistance(speakerPose.getTranslation());

        double RPM = Constants.ShooterConstants.shooterMap.get(
                Units.metersToFeet(distanceToSpeaker3 + ShooterConstants.CONSTANT_DISTANCE_ADD));
        SmartDashboard.putNumber("Calculated RPM", RPM);
        return RPM;
    }

    public static Rotation2d calculateAngleToSpeaker(Supplier<Pose2d> poseSupplier, Supplier<Translation2d> speeds) {
        Pose2d speakerPose;
        Rotation2d angle;
        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            speakerPose = Constants.speakerConstants.speakerLocBlue;
        } else {
            speakerPose = Constants.speakerConstants.speakerLocRed;
        }
        double distanceToSpeaker = poseSupplier.get().getTranslation().getDistance(speakerPose.getTranslation())
                + ShooterConstants.CONSTANT_DISTANCE_ADD;
        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            angle = Constants.speakerConstants
                    .speakerLocBlue
                    .getTranslation()
                    .minus(poseSupplier
                            .get()
                            .getTranslation()
                            .plus(speeds.get().times(distanceToSpeaker * Constants.shootMoveMultiplier)))
                    .getAngle()
                    .times(-1.0)
                    .plus(new Rotation2d(Units.degreesToRadians(180.0 + 6.0)));
        } else {
            angle = Constants.speakerConstants
                    .speakerLocRed
                    .getTranslation()
                    .minus(poseSupplier
                            .get()
                            .getTranslation()
                            .plus(speeds.get().times(distanceToSpeaker * Constants.shootMoveMultiplier)))
                    .getAngle()
                    .times(-1.0)
                    .plus(new Rotation2d(Units.degreesToRadians(6.0)));
        }
        SmartDashboard.putNumber("target Angle", angle.getDegrees());
        return angle;
        // // ALTERNATE LOGIC
        // return speakerPose
        //         .getTranslation()
        //         .minus(poseSupplier.get().getTranslation())
        //         .getAngle();

        // double robotAimAngle = Math.atan2(
        //         speakerPose.getY() - poseSupplier.get().getY(),
        //         speakerPose.getX() - poseSupplier.get().getX()); // Robot Angle

        // return new Rotation2d(robotAimAngle);
    }
}
