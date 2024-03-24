// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LightningShuffleboard;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    /** Creates a new Arm.  */
    private final ArmIO io;

    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public Arm(ArmIO ArmIO) {
        this.io = ArmIO;

        // LightningShuffleboard.setDoubleSupplier("shoulder", "shoulder left pose", io::getShoulderLeftPosition);
        // LightningShuffleboard.setDoubleSupplier("shoulder", "shoulder right pose", io::getShoulderRightPosition);
        // LightningShuffleboard.setDoubleSupplier("elbow", "elbow left pose", io::getElbowLeftPosition);
        // LightningShuffleboard.setDoubleSupplier("elbow", "elbow right pose", io::getElbowRightPosition);
        // LightningShuffleboard.setDoubleSupplier("arm", "gripper pose", io::getGripperPosition);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        // io.callThisInPeriodic();

        // io.setElbowPosition(LightningShuffleboard.getDouble("elbow", "set pose", 0));
        // io.setShoulderPosition(LightningShuffleboard.getDouble("shoulder", "set pose", 0));

    }

    public void setGains(boolean climbing) {
        io.setGains(climbing);
    }

    public void setArmVelocity(Translation2d velocity) {
        io.setArmVelocity(velocity);
    }

    public void setArmPosition(Translation2d position, boolean inBend) {
        io.setArmPosition(position, inBend);
    }

    public void setGripperPower(double power) {
        io.setGripperPower(power);
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

    public double getGripperPosition() {
        return io.getGripperPosition();
    }
}
