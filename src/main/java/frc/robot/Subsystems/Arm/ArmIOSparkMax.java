// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

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

    private boolean inBend;

    /** Creates a new ArmIOSparkMax. */
    public ArmIOSparkMax() {
        shoulderLeft = new CANSparkMax(Constants.shoulderLeftID, MotorType.kBrushless);
        // shoulderRight = new CANSparkMax(Constants.shoulderRightID, MotorType.kBrushless);
        elbowLeft = new CANSparkMax(Constants.elbowLeftID, MotorType.kBrushless);
        // elbowRight = new CANSparkMax(Constants.elbowRightID, MotorType.kBrushless);
        // wrist = new CANSparkMax(Constants.wristID, MotorType.kBrushless);
        // gripper = new CANSparkMax(Constants.gripperID, MotorType.kBrushless);

        shoulderLeft.setInverted(true);
        // shoulderRight.setInverted(true);
        // elbowRight.setInverted(false);
        elbowLeft.setInverted(true);
        shoulderLeftEncoder = shoulderLeft.getEncoder();
        // shoulderRightEncoder = shoulderRight.getEncoder();
        elbowLeftEncoder = elbowLeft.getEncoder();
        // elbowRightEncoder = elbowRight.getEncoder();
        // wristEncoder = wrist.getEncoder();
        // gripperEncoder = gripper.getEncoder();
        shoulderLeftEncoder.setPositionConversionFactor(ArmConstants.shoulderRadPerRot);
        // shoulderRightEncoder.setPositionConversionFactor(ArmConstants.shoulderRadPerRot);
        elbowLeftEncoder.setPositionConversionFactor(ArmConstants.elbowRadPerRot);
        // elbowRightEncoder.setPositionConversionFactor(ArmConstants.elbowRadPerRot);
        // wristEncoder.setPositionConversionFactor(ArmConstants.wristRadPerRot);



        shoulderLeftEncoder.setPosition(ArmConstants.shoulderOffset);
        // shoulderRightEncoder.setPosition(ArmConstants.shoulderOffset);
        elbowLeftEncoder.setPosition(ArmConstants.elbowOffset);
        // elbowRightEncoder.setPosition(ArmConstants.elbowOffset);
        // wristEncoder.setPosition(0);
        // gripperEncoder.setPosition(0);

        shoulderLeftController = shoulderLeft.getPIDController();
        // shoulderRightController = shoulderRight.getPIDController();
        elbowLeftController = elbowLeft.getPIDController();
        // elbowRightController = elbowRight.getPIDController();
        // wristController = wrist.getPIDController();

        inBend = false;
        resetEncoders();
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

        // inputs.wristPosition = Units.rotationsToRadians(wristEncoder.getPosition());
        // inputs.wristVelocity = Units.rotationsPerMinuteToRadiansPerSecond(wristEncoder.getVelocity());
        // inputs.wristAppliedVolts = wrist.getAppliedOutput() * wrist.getBusVoltage();
        // inputs.wristCurrentAmps = new double[] {wrist.getOutputCurrent()};

        // inputs.gripperPosition = Units.rotationsToRadians(gripperEncoder.getPosition());
        // inputs.gripperVelocity = Units.rotationsPerMinuteToRadiansPerSecond(gripperEncoder.getVelocity());
        // inputs.gripperAppliedVolts = gripper.getAppliedOutput() * gripper.getBusVoltage();
        // inputs.gripperCurrentAmps = new double[] {gripper.getOutputCurrent()};
    }

    public void setArmVelocity(Translation2d velocity) {
        setArmPosition(getArmPosition().plus(velocity), inBend);
    }

    public void setArmPosition(Translation2d position, boolean inBend) {
        this.inBend = inBend;
        double distance = position.getNorm();

        double BaseAngleArmDiff = Math.acos(((distance * distance)
                        + (ArmConstants.baseStageLength * ArmConstants.baseStageLength)
                        - (ArmConstants.secondStageLength * ArmConstants.secondStageLength))
                / (2 * distance * ArmConstants.baseStageLength));
        double SecondAngleArmDiff = Math.acos(((distance * distance)
                        - (ArmConstants.baseStageLength * ArmConstants.baseStageLength)
                        + (ArmConstants.secondStageLength * ArmConstants.secondStageLength))
                / (2 * distance * ArmConstants.secondStageLength));

        double shoulderPosition = position.getAngle().getRadians() + (BaseAngleArmDiff * (inBend ? 1 : -1));
        double elbowPosition = position.getAngle().getRadians() + (SecondAngleArmDiff * (inBend ? -1 : 1));
        setShoulderPosition(shoulderPosition);
        setElbowPosition(elbowPosition);
        SmartDashboard.putNumber("shoulderPosition error", shoulderPosition - getShoulderPosition());
        SmartDashboard.putNumber("elbowPosition error", elbowPosition - getElbowPosition());

    //    setShoulderPosition(position.getX());
      //  setElbowPosition(position.getY());
        // System.out.println("shoulder" + (angle - BaseAngleArmDiff));
        // System.out.println("elbow" + (angle + SecondAngleArmDiff));
    }

    public void resetEncoders() {
        shoulderLeft.setSmartCurrentLimit(ArmConstants.currentLimitShoulder);
        elbowLeft.setSmartCurrentLimit(ArmConstants.currentLimitElbow);
        shoulderLeftController.setP(ArmConstants.shoulderP,0);
        shoulderLeftController.setI(ArmConstants.shoulderI,0);
        shoulderLeftController.setD(ArmConstants.shoulderD,0);
        elbowLeftController.setI(ArmConstants.elbowP,0);
        elbowLeftController.setP(ArmConstants.elbowI,0);
        elbowLeftController.setD(ArmConstants.elbowD,0);
        shoulderLeftEncoder.setPosition(ArmConstants.shoulderOffset);
        // shoulderRightEncoder.setPosition(ArmConstants.shoulderOffset);
        elbowLeftEncoder.setPosition(ArmConstants.elbowOffset);
        elbowLeft.setClosedLoopRampRate(ArmConstants.rampRateElbow);
        shoulderLeft.setClosedLoopRampRate(ArmConstants.rampRateShoulder);
        // elbowRightEncoder.setPosition(ArmConstants.elbowOffset);
        // wristEncoder.setPosition(0);
        // gripperEncoder.setPosition(0);
        elbowLeftController.setOutputRange(-ArmConstants.maxPowerElbow, ArmConstants.maxPowerElbow);
        shoulderLeftController.setOutputRange(-ArmConstants.maxPowerShoulder, ArmConstants.maxPowerShoulder);
    }

    public Translation2d getArmPosition() {
        Translation2d jointPos = new Translation2d(
                Math.cos(getShoulderPosition()) * ArmConstants.baseStageLength,
                Math.sin(getShoulderPosition()) * ArmConstants.baseStageLength);
        Translation2d jointToEndPos = new Translation2d(
                Math.cos(getElbowPosition()) * ArmConstants.secondStageLength,
                Math.sin(getElbowPosition()) * ArmConstants.secondStageLength);
        SmartDashboard.putNumber("arm x position", jointPos.plus(jointToEndPos).getX());
        SmartDashboard.putNumber("arm y position", jointPos.plus(jointToEndPos).getY());
        return jointPos.plus(jointToEndPos);
    }

    public void setShoulderPosition(double position) {
        shoulderLeftController.setReference(position, ControlType.kPosition);
        // shoulderRightController.setReference(position, ControlType.kPosition);
    }

    public void setShoulderVelocity(double velocity) {
        shoulderLeftController.setReference(velocity, ControlType.kVelocity);
        // shoulderRightController.setReference(velocity, ControlType.kVelocity);
    }

    public double getShoulderPosition() {
        SmartDashboard.putNumber("shoulder position", shoulderLeftEncoder.getPosition());
        return shoulderLeftEncoder.getPosition();
    }

    public void setElbowPosition(double position) {
        // position -= ((ArmConstants.virtual4BarGearRatio - 1) * (getShoulderPosition() - ArmConstants.shoulderOffset));
        elbowLeftController.setReference(position - shoulderLeftEncoder.getPosition()*24.0/42.0, ControlType.kPosition);
        // elbowRightController.setReference(position, ControlType.kPosition);
    }

    public void setElbowVelocity(double velocity) {
        elbowLeftController.setReference(velocity, ControlType.kVelocity);
        // elbowRightController.setReference(velocity, ControlType.kVelocity);
    }

    public double getElbowPosition() {
        double elbowPose = elbowLeftEncoder.getPosition();
        elbowPose += shoulderLeftEncoder.getPosition()*24.0/42.0;
        SmartDashboard.putNumber("elbow position", elbowPose);
        SmartDashboard.putNumber("raw elbow position", elbowLeftEncoder.getPosition());
        // SmartDashboard.putNumber("elbow adjustment factor", shoulderLeftEncoder.getPosition()*24.0/42.0);
        // SmartDashboard.putNumber("elbow to shoulder", elbowPose - shoulderLeftEncoder.getPosition());
        return elbowPose;
      //          + ((ArmConstants.virtual4BarGearRatio - 1) * (getShoulderPosition() - ArmConstants.shoulderOffset));
    }

    // public void setWristPosition(double position) {
    //     wristController.setReference(position, ControlType.kPosition);
    // }

    // public double getWristPosition() {
    //     return wristEncoder.getPosition();
    // }

    // public void setGripper(double power) {
    //     gripper.set(power);
    // }
}
