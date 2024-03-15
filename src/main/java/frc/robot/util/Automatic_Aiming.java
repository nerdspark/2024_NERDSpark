package frc.robot.util;

public class Automatic_Aiming {

    public static double SweepCalculator(
            double robotPoseX, double robotPoseY, double speakerPoseX, double speakerPoseY) {

        double sweepAngle = 0;
        double sweepOffset = 0;

        sweepAngle = Math.atan((robotPoseY - speakerPoseY + sweepOffset) / (robotPoseX - speakerPoseX));

        return sweepAngle;
    }

    public static double shooterAngleCalculator() {
        double shooterAngle = 0; // TODO: code this when shooter is built, add error calculation

        return shooterAngle;
    }
}
