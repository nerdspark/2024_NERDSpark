// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

public class ArmIOSparkMax implements ArmIO {
    private CANSparkMax shoulderLeft;
    private CANSparkMax shoulderRight;
    private CANSparkMax elbowLeft;
    private CANSparkMax elbowRight;
    private CANSparkMax wrist;
    // private CANSparkMax gripper;

    private RelativeEncoder shoulderLeftEncoder;
    private RelativeEncoder shoulderRightEncoder;
    private RelativeEncoder elbowLeftEncoder;
    private RelativeEncoder elbowRightEncoder;
    private RelativeEncoder wristEncoder;
    // private RelativeEncoder gripperEncoder;

    private PIDController shoulderLeftController;
    private PIDController shoulderRightController;
    private PIDController elbowLeftController;
    private PIDController elbowRightController;

    private SparkPIDController wristController;

    private boolean inBend;
    // private PIDController shoulderController; //
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#introduction-to-dc-motor-feedforward
    // private PIDController elbowController;
    private ArmFeedforward shoulderLeftFeedforward;
    private ArmFeedforward shoulderRightFeedforward;
    private ArmFeedforward elbowLeftFeedforward;
    private ArmFeedforward
            elbowRightFeedforward; // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html#armfeedforward

    /** Creates a new ArmIOSparkMax. */
    public ArmIOSparkMax() {
        shoulderLeft = new CANSparkMax(Constants.shoulderLeftID, MotorType.kBrushless);
        shoulderRight = new CANSparkMax(Constants.shoulderRightID, MotorType.kBrushless);
        elbowLeft = new CANSparkMax(Constants.elbowLeftID, MotorType.kBrushless);
        elbowRight = new CANSparkMax(Constants.elbowRightID, MotorType.kBrushless);
        wrist = new CANSparkMax(Constants.wristID, MotorType.kBrushless);
        // gripper = new CANSparkMax(Constants.gripperID, MotorType.kBrushless);

        shoulderLeft.setInverted(true);
        shoulderRight.setInverted(false);
        elbowRight.setInverted(false);
        elbowLeft.setInverted(true);
        shoulderLeftEncoder = shoulderLeft.getEncoder();
        shoulderRightEncoder = shoulderRight.getEncoder();
        elbowLeftEncoder = elbowLeft.getEncoder();
        elbowRightEncoder = elbowRight.getEncoder();
        wristEncoder = wrist.getEncoder();
        // gripperEncoder = gripper.getEncoder();
        shoulderLeftEncoder.setPositionConversionFactor(ArmConstants.shoulderRadPerRot);
        shoulderRightEncoder.setPositionConversionFactor(ArmConstants.shoulderRadPerRot);
        elbowLeftEncoder.setPositionConversionFactor(ArmConstants.elbowRadPerRot);
        elbowRightEncoder.setPositionConversionFactor(ArmConstants.elbowRadPerRot);
        wristEncoder.setPositionConversionFactor(ArmConstants.wristRadPerRot);

        shoulderLeftEncoder = shoulderLeft.getEncoder();
        shoulderRightEncoder = shoulderRight.getEncoder();
        elbowLeftEncoder = elbowLeft.getEncoder();
        elbowRightEncoder = elbowRight.getEncoder();
        wristEncoder = wrist.getEncoder();

        shoulderLeftEncoder.setPosition(ArmConstants.shoulderOffset);
        shoulderRightEncoder.setPosition(ArmConstants.shoulderOffset);
        elbowLeftEncoder.setPosition(ArmConstants.elbowOffset);
        elbowRightEncoder.setPosition(ArmConstants.elbowOffset);
        wristEncoder.setPosition(0);
        // gripperEncoder.setPosition(0);

        shoulderLeftController =
                new PIDController(ArmConstants.shoulderP, ArmConstants.shoulderI, ArmConstants.shoulderD);
        shoulderRightController =
                new PIDController(ArmConstants.shoulderP, ArmConstants.shoulderI, ArmConstants.shoulderD);
        elbowLeftController = new PIDController(ArmConstants.elbowP, ArmConstants.elbowI, ArmConstants.elbowD);
        elbowRightController = new PIDController(ArmConstants.elbowP, ArmConstants.elbowI, ArmConstants.elbowD);
        wristController = wrist.getPIDController();

        inBend = false;
        resetEncoders();
        shoulderLeftFeedforward = new ArmFeedforward(
                ArmConstants.shoulderS, ArmConstants.shoulderG, ArmConstants.shoulderV, ArmConstants.shoulderA);
        shoulderRightFeedforward = new ArmFeedforward(
                ArmConstants.shoulderS, ArmConstants.shoulderG, ArmConstants.shoulderV, ArmConstants.shoulderA);
        elbowLeftFeedforward =
                new ArmFeedforward(ArmConstants.elbowS, ArmConstants.elbowG, ArmConstants.elbowV, ArmConstants.elbowA);
        elbowRightFeedforward =
                new ArmFeedforward(ArmConstants.elbowS, ArmConstants.elbowG, ArmConstants.elbowV, ArmConstants.elbowA);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.shoulderLeftPosition = Units.rotationsToRadians(shoulderLeftEncoder.getPosition());
        inputs.shoulderLeftVelocity = Units.rotationsPerMinuteToRadiansPerSecond(shoulderLeftEncoder.getVelocity());
        inputs.shoulderLeftAppliedVolts = shoulderLeft.getAppliedOutput() * shoulderLeft.getBusVoltage();
        inputs.shoulderLeftCurrentAmps = new double[] {shoulderLeft.getOutputCurrent()};

        inputs.elbowLeftPosition = Units.rotationsToRadians(elbowLeftEncoder.getPosition());
        inputs.elbowLeftVelocity = Units.rotationsPerMinuteToRadiansPerSecond(elbowLeftEncoder.getVelocity());
        inputs.elbowLeftAppliedVolts = elbowLeft.getAppliedOutput() * elbowLeft.getBusVoltage();
        inputs.elbowLeftCurrentAmps = new double[] {elbowLeft.getOutputCurrent()};

        inputs.shoulderRightPosition = Units.rotationsToRadians(shoulderLeftEncoder.getPosition());
        inputs.shoulderRightVelocity = Units.rotationsPerMinuteToRadiansPerSecond(shoulderLeftEncoder.getVelocity());
        inputs.shoulderRightAppliedVolts = shoulderLeft.getAppliedOutput() * shoulderLeft.getBusVoltage();
        inputs.shoulderRightCurrentAmps = new double[] {shoulderLeft.getOutputCurrent()};

        inputs.elbowRightPosition = Units.rotationsToRadians(elbowLeftEncoder.getPosition());
        inputs.elbowRightVelocity = Units.rotationsPerMinuteToRadiansPerSecond(elbowLeftEncoder.getVelocity());
        inputs.elbowRightAppliedVolts = elbowLeft.getAppliedOutput() * elbowLeft.getBusVoltage();
        inputs.elbowRightCurrentAmps = new double[] {elbowLeft.getOutputCurrent()};

        inputs.wristPosition = Units.rotationsToRadians(wristEncoder.getPosition());
        inputs.wristVelocity = Units.rotationsPerMinuteToRadiansPerSecond(wristEncoder.getVelocity());
        inputs.wristAppliedVolts = wrist.getAppliedOutput() * wrist.getBusVoltage();
        inputs.wristCurrentAmps = new double[] {wrist.getOutputCurrent()};

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
    }

    public void resetEncoders() {
        shoulderLeft.setSmartCurrentLimit(ArmConstants.currentLimitShoulder);
        elbowLeft.setSmartCurrentLimit(ArmConstants.currentLimitElbow);
        shoulderLeftEncoder.setPosition(ArmConstants.shoulderOffset);
        shoulderRightEncoder.setPosition(ArmConstants.shoulderOffset);
        elbowLeftEncoder.setPosition(ArmConstants.elbowOffset);
        elbowLeft.setClosedLoopRampRate(ArmConstants.rampRateElbow);
        shoulderLeft.setClosedLoopRampRate(ArmConstants.rampRateShoulder);
        elbowRightEncoder.setPosition(ArmConstants.elbowOffset);
        wristEncoder.setPosition(0);
        // gripperEncoder.setPosition(0);
        // elbowLeftController.setOutputRange(-ArmConstants.maxPowerElbow, ArmConstants.maxPowerElbow);
        // shoulderLeftController.setOutputRange(-ArmConstants.maxPowerShoulder, ArmConstants.maxPowerShoulder);
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
        shoulderLeft.set(shoulderLeftController.calculate(getShoulderLeftPosition(), position)
                + shoulderLeftFeedforward.calculate(getShoulderLeftPosition(), getShoulderLeftVelocity()));
        shoulderRight.set(shoulderRightController.calculate(getShoulderRightPosition(), position)
                + shoulderRightFeedforward.calculate(getShoulderRightPosition(), getShoulderRightVelocity()));
    }

    // public void setShoulderVelocity(double velocity) {
    //     // shoulderLeftController.setReference(velocity, ControlType.kVelocity);
    //     // shoulderRightController.setReference(velocity, ControlType.kVelocity);
    //     shoulderLeftController.calculate(, velocity);
    // }

    public double getShoulderLeftPosition() {
        SmartDashboard.putNumber("shoulder l position", shoulderLeftEncoder.getPosition());
        return shoulderLeftEncoder.getPosition();
    }

    public double getShoulderRightPosition() {
        SmartDashboard.putNumber("shoulder r position", shoulderRightEncoder.getPosition());
        return shoulderRightEncoder.getPosition();
    }

    public void setElbowPosition(double position) {
        elbowLeft.set(elbowLeftController.calculate(getElbowLeftPosition(), position)
                + elbowLeftFeedforward.calculate(getElbowLeftPosition(), getElbowLeftVelocity()));
        elbowRight.set(elbowRightController.calculate(getElbowRightPosition(), position)
                + elbowRightFeedforward.calculate(getElbowRightPosition(), getElbowRightVelocity()));
    }

    // public void setElbowVelocity(double velocity) {
    //     elbowLeftController.setReference(velocity, ControlType.kVelocity);
    //     // elbowRightController.setReference(velocity, ControlType.kVelocity);
    // }
    public double getElbowLeftVelocity() {
        return elbowLeftEncoder.getVelocity();
    }

    public double getShoulderLeftVelocity() {
        return shoulderLeftEncoder.getVelocity();
    }

    public double getElbowRightVelocity() {
        return elbowRightEncoder.getVelocity();
    }

    public double getShoulderRightVelocity() {
        return shoulderRightEncoder.getVelocity();
    }

    public double getElbowLeftPosition() {
        double elbowPose = elbowLeftEncoder.getPosition();
        elbowPose += shoulderLeftEncoder.getPosition() * 24.0 / 42.0;
        SmartDashboard.putNumber("elbow l position", elbowPose);
        SmartDashboard.putNumber("raw elbow l position", elbowLeftEncoder.getPosition());
        // SmartDashboard.putNumber("elbow adjustment factor", shoulderLeftEncoder.getPosition()*24.0/42.0);
        // SmartDashboard.putNumber("elbow to shoulder", elbowPose - shoulderLeftEncoder.getPosition());
        return elbowPose;
        //          + ((ArmConstants.virtual4BarGearRatio - 1) * (getShoulderPosition() - ArmConstants.shoulderOffset));
    }

    public double getElbowRightPosition() {
        double elbowPose = elbowRightEncoder.getPosition();
        elbowPose += shoulderRightEncoder.getPosition() * 24.0 / 42.0;
        SmartDashboard.putNumber("elbow r position", elbowPose);
        SmartDashboard.putNumber("raw elbow r position", elbowRightEncoder.getPosition());
        // SmartDashboard.putNumber("elbow adjustment factor", shoulderLeftEncoder.getPosition()*24.0/42.0);
        // SmartDashboard.putNumber("elbow to shoulder", elbowPose - shoulderLeftEncoder.getPosition());
        return elbowPose;
        //          + ((ArmConstants.virtual4BarGearRatio - 1) * (getShoulderPosition() - ArmConstants.shoulderOffset));
    }

    public void setWristPosition(double position) {
        wristController.setReference(position, ControlType.kPosition);
    }

    public double getWristPosition() {
        return wristEncoder.getPosition();
    }

    //     public void setGripper(double power) {
    //         gripper.set(power);
    //     }
}