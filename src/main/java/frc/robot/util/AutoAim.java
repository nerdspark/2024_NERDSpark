package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants.AmpSetpoints;
import frc.robot.Constants.BiasConstants;
import frc.robot.Constants.FixedShotConstants;
import frc.robot.Constants.FourBarConstants;
import frc.robot.Constants.ShooterConstants;
import java.util.function.Supplier;

public class AutoAim {
    Supplier<Pose2d> poseSupplier;
    private static double distanceOffset = 0;

    public AutoAim() {}

    public static boolean shootWhenNearSpeaker(Supplier<Pose2d> poseSupplier) {
        Pose2d speakerPose;
        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            speakerPose = Constants.SpeakerConstants.speakerLocBlue;
        } else {
            speakerPose = Constants.SpeakerConstants.speakerLocRed;
        }
        double distanceToSpeaker = poseSupplier.get().getTranslation().getDistance(speakerPose.getTranslation());
        return distanceToSpeaker < FixedShotConstants.drivingLongShotDistance;
    }

    public static boolean shootWhenNearAmp(Supplier<Pose2d> poseSupplier, Supplier<Translation2d> speeds) {
        Translation2d m_speeds = speeds.get();
        if (m_speeds.getNorm() > AmpSetpoints.armAutoAmpMaxSpeed) {
            m_speeds = m_speeds.times(AmpSetpoints.armAutoAmpMaxSpeed / m_speeds.getNorm());
        }

        Pose2d m_pose = poseSupplier
                .get()
                .transformBy(
                        new Transform2d(m_speeds.times(AmpSetpoints.armAutoAmpMovingMultiplier), new Rotation2d()));
        Translation2d ampPose = new Translation2d(
                poseSupplier.get().getX() < FieldConstants.fieldLength / 2.0
                        ? AmpSetpoints.armAutoAmpTargetPoseX
                        : 16.54 - AmpSetpoints.armAutoAmpTargetPoseX,
                AmpSetpoints.armAutoAmpTargetPoseY);

        double distanceX = Math.abs(m_pose.getX() - ampPose.getX());
        double distanceY = Math.abs(m_pose.getY() - ampPose.getY());
        return distanceX < AmpSetpoints.armAutoAmpToleranceX && distanceY < AmpSetpoints.armAutoAmpToleranceY;
    }

    public static double calculateYDistanceToAmp(Supplier<Pose2d> poseSupplier, Supplier<Translation2d> speeds) {
        Translation2d m_speeds = speeds.get();
        if (m_speeds.getNorm() > AmpSetpoints.armAutoAmpMaxSpeed) {
            m_speeds = m_speeds.times(AmpSetpoints.armAutoAmpMaxSpeed / Math.abs(m_speeds.getNorm()));
        }

        Translation2d m_pose = poseSupplier
                .get()
                .transformBy(new Transform2d(
                        m_speeds.times(AmpSetpoints.armAutoAmpArmPositioningMovingMultiplier), new Rotation2d()))
                .getTranslation();
        Translation2d ampPose = new Translation2d(
                poseSupplier.get().getX() < 8.27
                        ? AmpSetpoints.armAutoAmpTargetPoseX
                        : 16.54 - AmpSetpoints.armAutoAmpTargetPoseX,
                AmpSetpoints.armAutoAmpTargetPoseY);

        double distanceY = MathUtil.clamp(
                Units.metersToInches(m_pose.getY() - ampPose.getY()),
                -AmpSetpoints.ampMultiplierX,
                AmpSetpoints.ampMultiplierX);
        boolean close = m_pose.getDistance(ampPose) < AmpSetpoints.armAutoAmpArmUpTolerance;
        return close ? distanceY : 0;
    }

    public static double calculateFourBarPosition(Supplier<Pose2d> poseSupplier, Supplier<Translation2d> speeds) {
        Pose2d speakerPose;
        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            speakerPose = Constants.SpeakerConstants.speakerLocBlue;
        } else {
            speakerPose = Constants.SpeakerConstants.speakerLocRed;
        }
        double distanceToSpeaker = poseSupplier.get().getTranslation().getDistance(speakerPose.getTranslation());
        double distanceToSpeaker2 = poseSupplier
                .get()
                .getTranslation()
                .plus(speeds.get().times(distanceToSpeaker * ShooterConstants.shootMoveMultiplier))
                .getDistance(speakerPose.getTranslation());
        double distanceToSpeaker3 = poseSupplier
                .get()
                .getTranslation()
                .plus(speeds.get().times(distanceToSpeaker2 * ShooterConstants.shootMoveMultiplier))
                .getDistance(speakerPose.getTranslation());

        double angle = FourBarConstants.fourBarMap.get(
                distanceToSpeaker3 + distanceOffset + ShooterConstants.CONSTANT_DISTANCE_ADD);
        SmartDashboard.putNumber("Calculated 4Bar", angle);
        return angle;
    }

    public static double calculateShooterRPM(Supplier<Pose2d> poseSupplier, Supplier<Translation2d> speeds) {
        Pose2d speakerPose;
        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            speakerPose = Constants.SpeakerConstants.speakerLocBlue;
        } else {
            speakerPose = Constants.SpeakerConstants.speakerLocRed;
        }
        double distanceToSpeaker = poseSupplier.get().getTranslation().getDistance(speakerPose.getTranslation());
        double distanceToSpeaker2 = poseSupplier
                .get()
                .getTranslation()
                .plus(speeds.get().times(distanceToSpeaker * ShooterConstants.shootMoveMultiplier))
                .getDistance(speakerPose.getTranslation());
        double distanceToSpeaker3 = poseSupplier
                .get()
                .getTranslation()
                .plus(speeds.get().times(distanceToSpeaker2 * ShooterConstants.shootMoveMultiplier))
                .getDistance(speakerPose.getTranslation());

        double RPM = Constants.ShooterConstants.shooterMap.get(
                (distanceToSpeaker3 + distanceOffset + ShooterConstants.CONSTANT_DISTANCE_ADD));
        SmartDashboard.putNumber(
                "distance to speaker", distanceToSpeaker3 + distanceOffset + ShooterConstants.CONSTANT_DISTANCE_ADD);
        SmartDashboard.putNumber("Calculated RPM", RPM);
        return RPM;
    }

    public static Rotation2d calculateShooterSpin(Supplier<Double> RPM) {
        return new Rotation2d(Units.degreesToRadians(ShooterConstants.spinMap.get(Math.abs(RPM.get()))));
    }

    public static Rotation2d calculateAngleToSpeaker(Supplier<Pose2d> poseSupplier, Supplier<Translation2d> speeds) {
        Pose2d speakerPose;
        Rotation2d angle;
        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            speakerPose = Constants.SpeakerConstants.speakerLocBlue;
        } else {
            speakerPose = Constants.SpeakerConstants.speakerLocRed;
        }

        double distanceToSpeaker = poseSupplier.get().getTranslation().getDistance(speakerPose.getTranslation())
                + ShooterConstants.CONSTANT_DISTANCE_ADD;
        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            angle = Constants.SpeakerConstants.speakerLocBlue
                    .getTranslation()
                    .minus(poseSupplier
                            .get()
                            .getTranslation()
                            .plus(speeds.get().times(distanceToSpeaker * ShooterConstants.shootMoveMultiplier)))
                    .getAngle()
                    .times(-1.0)
                    .plus(new Rotation2d(Units.degreesToRadians(180.0)));
            // .minus(new Rotation2d().fromDegrees(ShooterConstants.spinMap.get(calculateShooterRPM(poseSupplier,
            // speeds))));
        } else {
            angle = Constants.SpeakerConstants.speakerLocRed
                    .getTranslation()
                    .minus(poseSupplier
                            .get()
                            .getTranslation()
                            .plus(speeds.get().times(distanceToSpeaker * ShooterConstants.shootMoveMultiplier)))
                    .getAngle()
                    .times(-1.0);
            // .minus(new Rotation2d().fromDegrees(ShooterConstants.spinMap.get(calculateShooterRPM(poseSupplier,
            // speeds))));
        }

        SmartDashboard.putNumber("target Angle", angle.getDegrees());
        return angle;
    }

    public void incDist() {
        distanceOffset += BiasConstants.distanceBiasIncrement;
    }

    public void decDist() {
        distanceOffset -= BiasConstants.distanceBiasIncrement;
    }
}
