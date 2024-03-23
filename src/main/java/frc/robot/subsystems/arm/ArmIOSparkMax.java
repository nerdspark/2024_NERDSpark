// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmGains;
import frc.robot.util.LightningShuffleboard;
import frc.robot.Constants.RobotMap;

public class ArmIOSparkMax implements ArmIO {
    private TalonFX shoulderRight;
    private TalonFX shoulderLeft;
    private TalonFX elbowLeft;
    private TalonFX elbowRight;
    private CANSparkMax gripper;

    private RelativeEncoder gripperEncoder;

    // private PIDController shoulderLeftController;
    // private PIDController shoulderRightController;
    // private PIDController elbowLeftController;
    // private PIDController elbowRightController;

    private boolean inBend;
    // private PIDController shoulderController;
    // private PIDController elbowController;
    // private ArmFeedforward shoulderLeftFeedforward;
    // private ArmFeedforward shoulderRightFeedforward;
    // private ArmFeedforward elbowLeftFeedforward;
    // private ArmFeedforward elbowRightFeedforward;
    private ArmGains armGains = new ArmGains();


    public ArmIOSparkMax() {

        shoulderLeft = new TalonFX(RobotMap.shoulderLeftID, "canivore1");
        shoulderRight = new TalonFX(RobotMap.shoulderRightID, "canivore1");
        elbowLeft = new TalonFX(RobotMap.elbowLeftID, "canivore1");
        elbowRight = new TalonFX(RobotMap.elbowRightID, "canivore1");

        TalonFXConfiguration shoulderconfig = new TalonFXConfiguration();
        TalonFXConfiguration elbowconfig = new TalonFXConfiguration();

        shoulderconfig.CurrentLimits =
                new CurrentLimitsConfigs().withStatorCurrentLimit(60).withStatorCurrentLimitEnable(true);
        shoulderconfig.Feedback = new FeedbackConfigs()
                .withFeedbackRotorOffset(ArmConstants.shoulderOffset)
                .withSensorToMechanismRatio(ArmConstants.shoulderRadPerRot);
        shoulderconfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.3 );
        shoulderconfig.Slot0 = new Slot0Configs()
                .withKP(armGains.shoulderP)
                .withKI(armGains.shoulderI)
                .withKD(armGains.shoulderD)
                .withKG(armGains.shoulderG)
                .withGravityType(GravityTypeValue.Arm_Cosine);

        shoulderLeft
                .getConfigurator()
                .apply(shoulderconfig.withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast)));
        shoulderRight
                .getConfigurator()
                .apply(shoulderconfig.withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast)));

        elbowconfig.CurrentLimits =
                new CurrentLimitsConfigs().withStatorCurrentLimit(60).withStatorCurrentLimitEnable(true);
        elbowconfig.Feedback = new FeedbackConfigs()
                .withFeedbackRotorOffset(ArmConstants.elbowOffset)
                .withSensorToMechanismRatio(ArmConstants.elbowRadPerRot);
        elbowconfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.1);
        elbowconfig.Slot0 = new Slot0Configs()
                .withKP(armGains.elbowP)
                .withKI(armGains.elbowI)
                .withKD(armGains.elbowD)
                .withKG(armGains.elbowG);

        elbowLeft
                .getConfigurator()
                .apply(elbowconfig.withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast)));
        elbowRight
                .getConfigurator()
                .apply(elbowconfig.withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast)));

        gripper = new CANSparkMax(RobotMap.gripperID, MotorType.kBrushless);

        shoulderLeft.setNeutralMode(NeutralModeValue.Coast);
        shoulderRight.setNeutralMode(NeutralModeValue.Coast);
        elbowLeft.setNeutralMode(NeutralModeValue.Coast);
        elbowRight.setNeutralMode(NeutralModeValue.Coast);

        shoulderLeft.setInverted(true);
        shoulderRight.setInverted(false);
        elbowRight.setInverted(false);
        elbowLeft.setInverted(true);
        gripper.setInverted(true);

        gripperEncoder = gripper.getEncoder();

        inBend = false;
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
        inputs.shoulderLeftPosition = shoulderLeft.getPosition().getValueAsDouble();
        inputs.shoulderLeftVelocity = shoulderLeft.getVelocity().getValueAsDouble();
        inputs.shoulderLeftAppliedVolts = shoulderLeft.getMotorVoltage().getValueAsDouble();
        // shoulderLeft.getAppliedOutput() * shoulderLeft.getBusVoltage();
        inputs.shoulderLeftCurrentAmps =
                new double[] {shoulderLeft.getStatorCurrent().getValueAsDouble()};

        inputs.elbowLeftPosition = elbowLeft.getPosition().getValueAsDouble();
        inputs.elbowLeftVelocity = elbowLeft.getVelocity().getValueAsDouble();
        inputs.elbowLeftAppliedVolts = elbowLeft.getMotorVoltage().getValueAsDouble();
        inputs.elbowLeftCurrentAmps = new double[] {elbowLeft.getStatorCurrent().getValueAsDouble()};

        inputs.shoulderRightPosition = (shoulderLeft.getPosition().getValueAsDouble());
        inputs.shoulderRightVelocity = (shoulderLeft.getVelocity()).getValueAsDouble();
        inputs.shoulderRightAppliedVolts = shoulderLeft.getMotorVoltage().getValueAsDouble();
        inputs.shoulderRightCurrentAmps =
                new double[] {shoulderLeft.getStatorCurrent().getValueAsDouble()};

        inputs.elbowRightPosition = elbowLeft.getPosition().getValueAsDouble();
        inputs.elbowRightVelocity = elbowLeft.getVelocity().getValueAsDouble();
        inputs.elbowRightAppliedVolts = elbowLeft.getMotorVoltage().getValueAsDouble();
        inputs.elbowRightCurrentAmps =
                new double[] {elbowLeft.getStatorCurrent().getValueAsDouble()};

        inputs.armX = getArmPosition().getX();
        inputs.armY = getArmPosition().getY();

        inputs.gripperPosition = getGripperPosition();
        inputs.gripperVelocity = gripperEncoder.getVelocity();
        inputs.gripperAppliedVolts = gripper.getAppliedOutput() * gripper.getBusVoltage();
        inputs.gripperCurrentAmps = new double[] {gripper.getOutputCurrent()};
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
        SmartDashboard.putNumber("shoulderPosition error", shoulderPosition - getShoulderLeftPosition());
        SmartDashboard.putNumber("elbow R Position error", elbowPosition - getElbowRightPosition());
        SmartDashboard.putNumber("elbow L Position error", elbowPosition - getElbowLeftPosition());
    }

    public void resetEncoders() {
        elbowLeft.setPosition(ArmConstants.elbowOffset);
        elbowRight.setPosition(ArmConstants.elbowOffset);
        shoulderLeft.setPosition(ArmConstants.shoulderOffset);
        shoulderRight.setPosition(ArmConstants.shoulderOffset);

        gripperEncoder.setPosition(0);
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
        position = position / (2d * Math.PI);

        position = MathUtil.clamp(position, 0, Math.PI);

        shoulderLeft.setControl(new PositionVoltage(position));
        shoulderRight.setControl(new PositionVoltage(position));
    }

    public double getShoulderLeftPosition() {
        // SmartDashboard.putNumber()"shoulder l position", shoulderLeft.getPosition().getValueAsDouble());
        return shoulderLeft.getPosition().getValueAsDouble() * (2d * Math.PI);
    }

    public double getShoulderRightPosition() {
        // SmartDashboard.putNumber("shoulder r position", shoulderRight.getPosition().getValueAsDouble());
        return shoulderRight.getPosition().getValueAsDouble() * (2d * Math.PI);
    }

    public void setElbowPosition(double position) {
        position = position / (2d * Math.PI);

        // elbowLeft.setControl(new PositionVoltage(position));
        // elbowRight.setControl(new PositionVoltage(position));
    }

    public double getElbowLeftPosition() {
        double elbowPose = elbowLeft.getPosition().getValueAsDouble() * (2d * Math.PI);
        elbowPose += getShoulderLeftPosition() * (1 - ArmConstants.virtual4BarGearRatio);
        SmartDashboard.putNumber("elbow l position", elbowPose);
        SmartDashboard.putNumber("raw elbow l position", elbowLeft.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("elbow adjustment factor", shoulderLeft.getPosition()*24.0/42.0);
        // SmartDashboard.putNumber("elbow to shoulder", elbowPose - shoulderLeft.getPosition());
        return elbowPose;
        //          + ((ArmConstants.virtual4BarGearRatio - 1) * (getShoulderPosition() - ArmConstants.shoulderOffset));
    }

    public double getElbowRightPosition() {
        double elbowPose = elbowRight.getPosition().getValueAsDouble() * (2d * Math.PI);
        elbowPose += getShoulderRightPosition() * (1 - ArmConstants.virtual4BarGearRatio);
        SmartDashboard.putNumber("elbow r position", elbowPose);
        SmartDashboard.putNumber(
                "raw elbow r position", elbowRight.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("elbow adjustment factor", shoulderLeft.getPosition()*24.0/42.0);
        // SmartDashboard.putNumber("elbow to shoulder", elbowPose - shoulderLeft.getPosition());
        return elbowPose;
        //          + ((ArmConstants.virtual4BarGearRatio - 1) * (getShoulderPosition() - ArmConstants.shoulderOffset));
    }

    /**
     * dont kill me
     */
    public void callThisInPeriodic() {
        Slot0Configs shoulCon = new Slot0Configs();

        shoulCon.kP = LightningShuffleboard.getDouble("shoulder", "kP", ArmGains.shoulderP);
        shoulCon.kI = LightningShuffleboard.getDouble("shoulder", "kI", ArmGains.shoulderI);
        shoulCon.kD = LightningShuffleboard.getDouble("shoulder", "kD", ArmGains.shoulderD);
        shoulCon.kA = LightningShuffleboard.getDouble("shoulder", "kG", ArmGains.shoulderG);


        Slot0Configs elbowCon = new Slot0Configs();

        elbowCon.kP = LightningShuffleboard.getDouble("elbow", "kP", ArmGains.elbowP);
        elbowCon.kI = LightningShuffleboard.getDouble("elbow", "kI", ArmGains.elbowI);
        elbowCon.kD = LightningShuffleboard.getDouble("elbow", "kD", ArmGains.elbowD);
        elbowCon.kA = LightningShuffleboard.getDouble("elbow", "kG", ArmGains.elbowG);

        // if(LightningShuffleboard.getBool("shoulder", "refresh", false)) {
            elbowLeft.getConfigurator().apply(elbowCon);
            elbowRight.getConfigurator().apply(elbowCon);
            shoulderLeft.getConfigurator().apply(shoulCon);
            shoulderRight.getConfigurator().apply(shoulCon);

            // LightningShuffleboard.setBool("shoulder", "refresh", false);
        // }

        LightningShuffleboard.setDouble("shoulder", "leftt out", shoulderLeft.getMotorVoltage().getValueAsDouble());
        LightningShuffleboard.setDouble("shoulder", "right out", shoulderRight.getMotorVoltage().getValueAsDouble());

        LightningShuffleboard.setDouble("elbow", "leftt out", elbowLeft.getMotorVoltage().getValueAsDouble());
        LightningShuffleboard.setDouble("elbow", "right out", elbowRight.getMotorVoltage().getValueAsDouble());
    }

    public void setGripperPower(double power) {
        gripper.set(power);
    }

    public double getGripperPosition() {
        return gripperEncoder.getPosition();
    }

    public double getElbowLeftVelocity() {
        return elbowLeft.getVelocity().getValueAsDouble();
    }

    public double getelbowLeftVelocity() {
        return shoulderLeft.getVelocity().getValueAsDouble();
    }

    public double getElbowRightVelocity() {
        return elbowRight.getVelocity().getValueAsDouble();
    }

    public double getShoulderRightVelocity() {
        return shoulderRight.getVelocity().getValueAsDouble();
    }
}
