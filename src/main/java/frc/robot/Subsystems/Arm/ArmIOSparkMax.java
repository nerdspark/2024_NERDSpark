// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm.ArmIO.ArmIOInputs;

public class ArmIOSparkMax implements ArmIO {
    private CANSparkMax shoulderLeft;
    private CANSparkMax shoulderRight;
    private CANSparkMax elbowLeft;
    private CANSparkMax elbowRight;
    private CANSparkMax wrist;
    private CANSparkMax gripper;

    private RelativeEncoder shoulderLeftEncoder;
    private RelativeEncoder shoulderRightEncoder;
    private RelativeEncoder elbowLeftEncoder;
    private RelativeEncoder elbowRightEncoder;
    private RelativeEncoder wristEncoder;
    private RelativeEncoder gripperEncoder;

    private SparkPIDController shoulderLeftController;
    private SparkPIDController shoulderRightController;
    private SparkPIDController elbowLeftController;
    private SparkPIDController elbowRightController;
    private SparkPIDController wristController;

    /** Creates a new ArmIOSparkMax. */
    public ArmIOSparkMax() {
        shoulderLeft = new CANSparkMax(Constants.shoulderLeftID, MotorType.kBrushless);
        shoulderRight = new CANSparkMax(Constants.shoulderRightID, MotorType.kBrushless);
        elbowLeft = new CANSparkMax(Constants.elbowLeftID, MotorType.kBrushless);
        elbowRight = new CANSparkMax(Constants.elbowRightID, MotorType.kBrushless);
        wrist = new CANSparkMax(Constants.wristID, MotorType.kBrushless);
        gripper = new CANSparkMax(Constants.gripperID, MotorType.kBrushless);

        shoulderLeft.setInverted(false);
        shoulderRight.setInverted(true);
        elbowRight.setInverted(false);
        elbowRight.setInverted(true);

        shoulderLeftEncoder = shoulderLeft.getEncoder();
        shoulderRightEncoder = shoulderRight.getEncoder();
        elbowLeftEncoder = elbowLeft.getEncoder();
        elbowRightEncoder = elbowRight.getEncoder();
        wristEncoder = wrist.getEncoder();
        gripperEncoder = gripper.getEncoder();

        shoulderLeftEncoder.setPosition(0);
        shoulderRightEncoder.setPosition(0);
        elbowLeftEncoder.setPosition(0);
        elbowRightEncoder.setPosition(0);
        wristEncoder.setPosition(0);
        gripperEncoder.setPosition(0);

        shoulderLeftController = shoulderLeft.getPIDController();
        shoulderRightController = shoulderRight.getPIDController();
        elbowLeftController = elbowLeft.getPIDController();
        elbowRightController = elbowRight.getPIDController();
        wristController = wrist.getPIDController();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.shoulderPosition = Units.rotationsToRadians(shoulderLeftEncoder.getPosition());
        inputs.shoulderVelocity = Units.rotationsPerMinuteToRadiansPerSecond(shoulderLeftEncoder.getVelocity());
        inputs.shoulderAppliedVolts = shoulderLeft.getAppliedOutput() * shoulderLeft.getBusVoltage();
        inputs.shoulderCurrentAmps = new double[] {shoulderLeft.getOutputCurrent()};

        inputs.elbowPosition = Units.rotationsToRadians(elbowLeftEncoder.getPosition());
        inputs.elbowVelocity = Units.rotationsPerMinuteToRadiansPerSecond(elbowLeftEncoder.getVelocity());
        inputs.elbowAppliedVolts = elbowLeft.getAppliedOutput() * elbowLeft.getBusVoltage();
        inputs.elbowCurrentAmps = new double[] {elbowLeft.getOutputCurrent()};

        inputs.wristPosition = Units.rotationsToRadians(wristEncoder.getPosition());
        inputs.wristVelocity = Units.rotationsPerMinuteToRadiansPerSecond(wristEncoder.getVelocity());
        inputs.wristAppliedVolts = wrist.getAppliedOutput() * wrist.getBusVoltage();
        inputs.wristCurrentAmps = new double[] {wrist.getOutputCurrent()};

        inputs.gripperPosition = Units.rotationsToRadians(gripperEncoder.getPosition());
        inputs.gripperVelocity = Units.rotationsPerMinuteToRadiansPerSecond(gripperEncoder.getVelocity());
        inputs.gripperAppliedVolts = gripper.getAppliedOutput() * gripper.getBusVoltage();
        inputs.gripperCurrentAmps = new double[] {gripper.getOutputCurrent()};
    }

    public void setShoulderPosition(double position) {
        shoulderLeftController.setReference(position, ControlType.kPosition);
        shoulderRightController.setReference(position, ControlType.kPosition);
    }

    public void setShoulderVelocity(double velocity) {
        shoulderLeftController.setReference(velocity, ControlType.kVelocity);
        shoulderRightController.setReference(velocity, ControlType.kVelocity);
    }

    public double getShoulderPosition() {
        return shoulderLeftEncoder.getPosition();
    }

    public void setElbowPosition(double position) {
        elbowLeftController.setReference(position, ControlType.kPosition);
        elbowRightController.setReference(position, ControlType.kPosition);
    }

    public void setElbowVelocity(double velocity) {
        elbowLeftController.setReference(velocity, ControlType.kVelocity);
        elbowRightController.setReference(velocity, ControlType.kVelocity);
    }

    public double getElbowPosition() {
        return elbowLeftEncoder.getPosition();
    }

    public void setWristPosition(double position) {
        wristController.setReference(position, ControlType.kPosition);
    }

    public double getWristPosition() {
        return wristEncoder.getPosition();
    }

    public void setGripper(double power) {
        gripper.set(power);
    }
}
