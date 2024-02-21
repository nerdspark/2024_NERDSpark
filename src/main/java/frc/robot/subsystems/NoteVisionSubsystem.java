// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class NoteVisionSubsystem extends SubsystemBase {

    public static NoteVisionSubsystem instance;
    private PhotonCamera noteCamera;
    private double yawVal = 0;
    private double pitchVal = 0;
    private double skewValue = 0;
    private double areaVal = 0;
    private boolean hasTarget = false;

    /** Creates a new ExampleSubsystem. */
    public NoteVisionSubsystem(String noteCameraName) {

        this.noteCamera = new PhotonCamera(noteCameraName);
    }

    public void setPipeLineIndex(int pipeLineIndex) {

        this.noteCamera.setPipelineIndex(pipeLineIndex);
    }

    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        var result = this.noteCamera.getLatestResult();

        if (result.hasTargets()) {
            this.yawVal = result.getBestTarget().getYaw();
            this.pitchVal = result.getBestTarget().getPitch();
            this.skewValue = result.getBestTarget().getSkew();
            this.areaVal = result.getBestTarget().getArea();
            this.hasTarget = true;
            // SmartDashboard.putNumber("Note Vision Camera Distance from target", getRange());
            // SmartDashboard.putNumber("Yaw", this.yawVal);
            // SmartDashboard.putNumber("Pitch", this.pitchVal);
            // SmartDashboard.putNumber("Skew", this.skewValue);
            // SmartDashboard.putNumber("Area", this.areaVal);
            // SmartDashboard.putBoolean("Has Target", hasTarget);
        } else {
            this.hasTarget = false;
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public double getYawVal() {
        return this.yawVal;
    }

    public double getPitchVal() {
        return this.pitchVal;
    }

    public double getAreVal() {
        return this.areaVal;
    }

    public double getSkewVal() {
        return this.skewValue;
    }

    public boolean hasTargets() {
        return this.hasTarget;
    }

    public List<PhotonTrackedTarget> getTargets() {
        List<PhotonTrackedTarget> targets = null;
        var results = this.noteCamera.getLatestResult();
        if (results.hasTargets()) {
            targets = results.getTargets();
        }

        return targets;
    }

    public double getRange() {

        double range = PhotonUtils.calculateDistanceToTargetMeters(
                Constants.VisionConstants.NOTE_HEIGHT_METERS,
                Constants.VisionConstants.NOTE_CAMERA_HEIGHT_METERS,
                Constants.VisionConstants.NOTE_CAMERA_PITCH_RADIANS,
                -Units.degreesToRadians(getPitchVal()));

        //  SmartDashboard.putNumber ("Cone Vision Camera Distance from target", range);

        return range;
    }
}
