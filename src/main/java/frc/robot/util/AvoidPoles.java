// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AvoidPolesConstants;

/** Add your docs here. */
public class AvoidPoles {
    public AvoidPoles() {}
    public static Translation2d adjustJoystick(Supplier<Pose2d> poseSupplier, Supplier<Translation2d> robotSpeeds, Supplier<Translation2d> joystickSpeeds, Supplier<Boolean> visionUpdated) {
        Translation2d pose = poseSupplier.get().getTranslation().plus(robotSpeeds.get().times(AvoidPolesConstants.lookAhead));
        // double speed = robotSpeeds.get().getNorm();
        Translation2d correction = new Translation2d();
        for (int i = 0; i < AvoidPolesConstants.Poles.length; i++) {
            Translation2d poseToPole = AvoidPolesConstants.Poles[i].minus(pose);
            double correctionSpeed = Math.pow(poseToPole.getNorm(), AvoidPolesConstants.distancePower);
            Rotation2d normalDirection = poseToPole/*robotSpeeds.get()*/.getAngle();//.plus(new Rotation2d(Math.PI/2.0));
            // if (poseToPole.getAngle().minus(robotSpeeds.get().getAngle()).getRotations() < 0) {
            //     correctionSpeed *= -1;
            // }
            if (poseToPole.getNorm() < AvoidPolesConstants.distanceThreshold) {
                correction = correction.plus(new Translation2d(correctionSpeed, normalDirection));
                SmartDashboard.putString("poseToPole", String.format("(%.3f, %.3f)", poseToPole.getX(), poseToPole.getY()));
            }
        }
        if (robotSpeeds.get().getNorm() > AvoidPolesConstants.robotSpeedThreshold && visionUpdated.get() && Timer.getMatchTime() > AvoidPolesConstants.timeThreshold) {
            correction = correction.times(joystickSpeeds.get().getNorm()).times(DriverStation.getAlliance().get().equals(Alliance.Red) ? AvoidPolesConstants.correctionGain : -AvoidPolesConstants.correctionGain);
        } else {
            correction = new Translation2d();
        } 
        SmartDashboard.putString("correction", String.format("(%.3f, %.3f)", correction.getX(), correction.getY()));
        Translation2d output = joystickSpeeds.get().plus(correction);
        return output;
    }
}
