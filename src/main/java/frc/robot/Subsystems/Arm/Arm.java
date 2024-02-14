// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    /** Creates a new Arm. */
    private final ArmIO io;

    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public Arm(ArmIO ArmIO) {
        this.io = ArmIO;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        // This method will be called once per scheduler run
    }

    public void setShoulderPosition(double position) {
        io.setShoulderPosition(position);
    }

    public void setArmPosition(Translation2d position, boolean inBend) {
        io.setArmPosition(position, inBend);
    }

    public void resetEncoders() {
        io.resetEncoders();
    }

    public double getShoulderPosition() {
        return io.getShoulderPosition();
    }

    public void setElbowPosition(double position) {
        io.setElbowPosition(position);
    }

    public double getElbowPosition() {
        return io.getElbowPosition();
    }

    public void setWristPosition(double position) {
        io.setWristPosition(position);
    }

    public double getWristPosition() {
        return io.getWristPosition();
    }

    public void setGripper(double power) {
        io.setGripper(power);
    }
    
    public Translation2d getArmPosition() {
        return io.getArmPosition();
    }
}
