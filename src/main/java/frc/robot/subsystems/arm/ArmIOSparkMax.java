// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmGains;
import frc.robot.Constants.ArmConstants.ArmGainsClimb;
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
    private static AbsoluteEncoder wristEncoderAbsolute;
    // private RelativeEncoder gripperEncoder;

    // private PIDController shoulderLeftController;
    // private PIDController shoulderRightController;
    // private PIDController elbowLeftController;
    // private PIDController elbowRightController;

    private SparkPIDController wristController;
    private double a;
    private boolean inBend;
    // private PIDController shoulderController; //
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#introduction-to-dc-motor-feedforward
    // private PIDController elbowController;
    // private ArmFeedforward shoulderLeftFeedforward;
    // private ArmFeedforward shoulderRightFeedforward;
    // private ArmFeedforward elbowLeftFeedforward;
    // private ArmFeedforward
    //         elbowRightFeedforward; //
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html#armfeedforward
    private ArmGains armGains = new ArmGains();

    /** Creates a new ArmIOSparkMax. */
    public ArmIOSparkMax() {
        shoulderLeft = new CANSparkMax(Constants.shoulderLeftID, MotorType.kBrushless);
        shoulderRight = new CANSparkMax(Constants.shoulderRightID, MotorType.kBrushless);
        elbowLeft = new CANSparkMax(Constants.elbowLeftID, MotorType.kBrushless);
        elbowRight = new CANSparkMax(Constants.elbowRightID, MotorType.kBrushless);
        wrist = new CANSparkMax(Constants.wristID, MotorType.kBrushless);
        wristEncoderAbsolute = elbowLeft.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        // gripper = new CANSparkMax(Constants.gripperID, MotorType.kBrushless);

        shoulderLeft.setInverted(true);
        shoulderRight.setInverted(false);
        elbowRight.setInverted(false);
        elbowLeft.setInverted(true);
        wrist.setInverted(false);
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

        wristEncoderAbsolute.setPositionConversionFactor(1.0);
        // SmartDashboard.putNumber("Absolute Encoder", wristEncoderAbsolute.getPosition());
        // SmartDashboard.updateValues();

        wristController = wrist.getPIDController();
        wristController.setP(ArmConstants.wristP);
        wristController.setI(ArmConstants.wristI);
        wristController.setD(ArmConstants.wristD);
        wristController.setOutputRange(-ArmConstants.wristMaxPower, ArmConstants.wristMaxPower);
        // wrist.setSoftLimit(null, 0)

        inBend = false;
        setGains(false);
        resetEncoders();
        // setArmGains(climbGains);

        // shoulderLeftEncoder.setPosition(ArmConstants.shoulderOffset);
        // shoulderRightEncoder.setPosition(ArmConstants.shoulderOffset);
        // elbowLeftEncoder.setPosition(ArmConstants.elbowOffset);
        // elbowRightEncoder.setPosition(ArmConstants.elbowOffset);
        // gripperEncoder.setPosition(0);

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

        inputs.wristPosition = Units.rotationsToRadians(wristEncoderAbsolute.getPosition() * 27);
        inputs.wristVelocity = Units.rotationsPerMinuteToRadiansPerSecond(wristEncoder.getVelocity());
        inputs.wristAppliedVolts = wrist.getAppliedOutput() * wrist.getBusVoltage();
        inputs.wristCurrentAmps = new double[] {wrist.getOutputCurrent()};

        // inputs.gripperPosition = Units.rotationsToRadians(gripperEncoder.getPosition());
        // inputs.gripperVelocity = Units.rotationsPerMinuteToRadiansPerSecond(gripperEncoder.getVelocity());
        // inputs.gripperAppliedVolts = gripper.getAppliedOutput() * gripper.getBusVoltage();
        // inputs.gripperCurrentAmps = new double[] {gripper.getOutputCurrent()};
    }

    public void setGains(boolean climbing) {
        armGains = climbing ? new ArmGainsClimb() : new ArmGains();
        if (!climbing) {
            armGains.elbowLeftController.setIZone(0.1);
            armGains.elbowRightController.setIZone(0.1);
            armGains.shoulderLeftController.setIZone(0.1);
            armGains.shoulderRightController.setIZone(0.1);
        } else {
            armGains.elbowLeftController.setIZone(0.25);
            armGains.elbowRightController.setIZone(0.25);
            armGains.shoulderLeftController.setIZone(0.25);
            armGains.shoulderRightController.setIZone(0.25);
        }
    }

    // public void setArmVelocity(Translation2d velocity) {
    //     setArmPosition(getArmPosition().plus(velocity), inBend);
    // }

    // private void resetArmGains(ArmGains gains) {
    //     shoulderLeftController.setPID(gains.shoulderP, gains.shoulderI, gains.shoulderD);
    //     shoulderRightController.setPID(gains.shoulderP, gains.shoulderI, gains.shoulderD);
    //     elbowLeftController.setPID(gains.elbowP, gains.elbowI, gains.elbowD);
    //     elbowRightController.setPID(gains.elbowP, gains.elbowI, gains.elbowD);

    //     shoulderLeftFeedforward = new ArmFeedforward(
    //             gains.shoulderS, gains.shoulderG, gains.shoulderV, gains.shoulderA);
    //     shoulderRightFeedforward = new ArmFeedforward(
    //             gains.shoulderS, gains.shoulderG, gains.shoulderV, gains.shoulderA);
    //     elbowLeftFeedforward =
    //             new ArmFeedforward(gains.elbowS, gains.elbowG, gains.elbowV, gains.elbowA);
    //     elbowRightFeedforward =
    //             new ArmFeedforward(gains.elbowS, gains.elbowG, gains.elbowV, gains.elbowA);

    //     elbowLeftController.setIZone(0.15);
    //     elbowRightController.setIZone(0.15);
    // }
    public void setArmPosition(Translation2d position, boolean inBend, double wrist) {
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
        setWristPosition(wrist);
        SmartDashboard.putNumber("shoulderPosition error", shoulderPosition - getShoulderLeftPosition());
        SmartDashboard.putNumber("elbow R Position error", elbowPosition - getElbowRightPosition());
        SmartDashboard.putNumber("elbow L Position error", elbowPosition - getElbowLeftPosition());
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
        wristEncoder.setPosition(ArmConstants.wristOffset
                + (-(wristEncoderAbsolute.getPosition() > 0.5
                                ? wristEncoderAbsolute.getPosition() - 1
                                : wristEncoderAbsolute.getPosition())
                        * 2
                        * Math.PI));
        a++;
        SmartDashboard.putNumber("a", a);

        // gripperEncoder.setPosition(0);
        // elbowLeftController.setOutputRange(-ArmConstants.maxPowerElbow, ArmConstants.maxPowerElbow);
        // shoulderLeftController.setOutputRange(-ArmConstants.maxPowerShoulder, ArmConstants.maxPowerShoulder);
    }

    public Translation2d getArmPosition() {
        Translation2d jointPos = new Translation2d(
                Math.cos(getShoulderLeftPosition()) * ArmConstants.baseStageLength,
                Math.sin(getShoulderLeftPosition()) * ArmConstants.baseStageLength);
        Translation2d jointToEndPos = new Translation2d(
                Math.cos(getElbowLeftPosition()) * ArmConstants.secondStageLength,
                Math.sin(getElbowLeftPosition()) * ArmConstants.secondStageLength);
        SmartDashboard.putNumber("arm x position", jointPos.plus(jointToEndPos).getX());
        SmartDashboard.putNumber("arm y position", jointPos.plus(jointToEndPos).getY());
        return jointPos.plus(jointToEndPos);
    }

    public void setShoulderPosition(double position) {
        shoulderLeft.set(armGains.shoulderLeftController.calculate(getShoulderLeftPosition(), position)
                + armGains.shoulderLeftFeedforward.calculate(getShoulderLeftPosition(), getShoulderLeftVelocity()));
        shoulderRight.set(armGains.shoulderRightController.calculate(getShoulderRightPosition(), position)
                + armGains.shoulderRightFeedforward.calculate(getShoulderRightPosition(), getShoulderRightVelocity()));
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
        elbowLeft.set(armGains.elbowLeftController.calculate(getElbowLeftPosition(), position)
                + armGains.elbowLeftFeedforward.calculate(getElbowLeftPosition(), getElbowLeftVelocity()));
        elbowRight.set(armGains.elbowRightController.calculate(getElbowRightPosition(), position)
                + armGains.elbowRightFeedforward.calculate(getElbowRightPosition(), getElbowRightVelocity()));
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
        elbowPose += shoulderLeftEncoder.getPosition() * (1 - ArmConstants.virtual4BarGearRatio);
        SmartDashboard.putNumber("elbow l position", elbowPose);
        SmartDashboard.putNumber("raw elbow l position", elbowLeftEncoder.getPosition());
        // SmartDashboard.putNumber("elbow adjustment factor", shoulderLeftEncoder.getPosition()*24.0/42.0);
        // SmartDashboard.putNumber("elbow to shoulder", elbowPose - shoulderLeftEncoder.getPosition());
        return elbowPose;
        //          + ((ArmConstants.virtual4BarGearRatio - 1) * (getShoulderPosition() - ArmConstants.shoulderOffset));
    }

    public double getElbowRightPosition() {
        double elbowPose = elbowRightEncoder.getPosition();
        elbowPose += shoulderRightEncoder.getPosition() * (1 - ArmConstants.virtual4BarGearRatio);
        SmartDashboard.putNumber("elbow r position", elbowPose);
        SmartDashboard.putNumber("raw elbow r position", elbowRightEncoder.getPosition());
        // SmartDashboard.putNumber("elbow adjustment factor", shoulderLeftEncoder.getPosition()*24.0/42.0);
        // SmartDashboard.putNumber("elbow to shoulder", elbowPose - shoulderLeftEncoder.getPosition());
        return elbowPose;
        //          + ((ArmConstants.virtual4BarGearRatio - 1) * (getShoulderPosition() - ArmConstants.shoulderOffset));
    }

    public void setWristPosition(double position) {
        // position -= getElbowLeftPosition();
        System.out.println("wrist raw" + wristEncoderAbsolute.getPosition());
        SmartDashboard.putNumber("AbsoluteEncoder", -wristEncoderAbsolute.getPosition());
        SmartDashboard.putNumber(
                "wristabsoluteencoder", ArmConstants.wristOffset + (-wristEncoderAbsolute.getPosition() / 2 * Math.PI));
        SmartDashboard.putNumber("wristRawEncoder", wristEncoder.getPosition());
        wristController.setReference(position, ControlType.kPosition);
    }

    public double getWristPosition() {
        SmartDashboard.putNumber("AbsoluteEncoder", -wristEncoderAbsolute.getPosition());
        SmartDashboard.putNumber(
                "wristabsoluteencoder", ArmConstants.wristOffset + (-wristEncoderAbsolute.getPosition() / 2 * Math.PI));
        SmartDashboard.putNumber("wristRawEncoder", wristEncoder.getPosition());
        return wristEncoder.getPosition();
    }

    //     public void setGripper(double power) {
    //         gripper.set(power);
    //     }
}
