// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

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
        Logger.processInputs("Arm", inputs);
        // This method will be called once per scheduler run
    }

    public void setArmVelocity(Translation2d velocity) {
        io.setArmVelocity(velocity);
    }

    public void setArmPosition(Translation2d position, boolean inBend, double wrist) {
        io.setArmPosition(position, inBend, wrist);
    }

    public void resetEncoders() {
        io.resetEncoders();
    }

    public Translation2d getArmPosition() {
        return io.getArmPosition();
    }

    public void setShoulderPosition(double position) {
        io.setShoulderPosition(position);
    }

    public double getShoulderLeftPosition() {
        return io.getShoulderLeftPosition();
    }

    public double getShoulderRightPosition() {
        return io.getShoulderRightPosition();
    }

    public void setElbowPosition(double position) {
        io.setElbowPosition(position);
    }

    public double getElbowLeftVelocity() {
        return io.getElbowLeftVelocity();
    }

    public double getShoulderLeftVelocity() {
        return io.getShoulderLeftVelocity();
    }

    public double getElbowRightVelocity() {
        return io.getElbowRightVelocity();
    }

    public double getShoulderRightVelocity() {
        return io.getShoulderRightVelocity();
    }

    public double getElbowLeftPosition() {
        return io.getElbowLeftPosition();
    }

    public double getElbowRightPosition() {
        return io.getElbowRightPosition();
    }

    public void setWristPosition(double position) {
        io.setWristPosition(position);
    }

    public double getWristPosition() {
        return io.getWristPosition();
    }
}
