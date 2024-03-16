// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.fourBar;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FourBarConstants;
import frc.robot.Constants.FourBarGains;
import frc.robot.Constants.RobotMap;

public class FourBarIOSparkMax implements FourBarIO {
    /** Creates a new FourBarIOSparkMax. */
    private CANSparkMax FourBarMotor1;

    private CANSparkMax FourBarMotor2;
    private RelativeEncoder FourBarEncoder1;

    private ArmFeedforward FourBarFeedforward1;
    private PIDController FourBarPIDController;
    
    private SparkPIDController BuiltInPID;

    public FourBarIOSparkMax() {

        FourBarMotor1 = new CANSparkMax(RobotMap.fourBarLeftID, CANSparkMax.MotorType.kBrushless);
        FourBarMotor2 = new CANSparkMax(RobotMap.fourBarRightID, CANSparkMax.MotorType.kBrushless);

        FourBarMotor1.setInverted(false);
        FourBarMotor2.setInverted(true);

        FourBarMotor1.setIdleMode(FourBarConstants.fourBarIdleMode);
        FourBarMotor2.setIdleMode(FourBarConstants.fourBarIdleMode);

        FourBarEncoder1 = FourBarMotor1.getEncoder();

        FourBarMotor1.setSmartCurrentLimit(FourBarConstants.currentLimit);
        FourBarMotor2.setSmartCurrentLimit(FourBarConstants.currentLimit);

        FourBarMotor1.setClosedLoopRampRate(FourBarConstants.closedLoopRampRate);
        FourBarMotor2.setClosedLoopRampRate(FourBarConstants.closedLoopRampRate);
        FourBarMotor1.setOpenLoopRampRate(FourBarConstants.openLoopRampRate);
        FourBarMotor2.setOpenLoopRampRate(FourBarConstants.openLoopRampRate);

        FourBarEncoder1.setPositionConversionFactor(FourBarConstants.positionConversionFactor);
        FourBarEncoder1.setVelocityConversionFactor(FourBarConstants.positionConversionFactor);
        FourBarEncoder1.setPosition(FourBarConstants.resetPosition);
        
        FourBarMotor2.follow(FourBarMotor1, true);

        FourBarPIDController = new PIDController(FourBarGains.kP, FourBarGains.kI, FourBarGains.kD);
        FourBarPIDController.setIZone(FourBarGains.kIZone);
        FourBarPIDController.setTolerance(FourBarConstants.fourBarTolerance);

        FourBarFeedforward1 = new ArmFeedforward(FourBarGains.kS, FourBarGains.kG, FourBarGains.kV, FourBarGains.kA);
        // fourBarFeedforward2 = new ArmFeedforward(Constants.FourBarGains.kS, Constants.FourBarGains.kG,
        // Constants.FourBarGains.kV, Constants.FourBarGains.kA);

        BuiltInPID = FourBarMotor1.getPIDController();
        BuiltInPID.setP(0.55);
        BuiltInPID.setI(0.000);
        BuiltInPID.setD(0.0);
        BuiltInPID.setIZone(1.0);
    }

    @Override
    @SuppressWarnings("static-access")
    public void updateInputs(FourBarIOInputs inputs) {
        inputs.FourBarPosition = FourBarEncoder1.getPosition();
        inputs.FourBarVelocity = FourBarEncoder1.getVelocity();
        inputs.FourBarAppliedVolts = FourBarMotor1.getAppliedOutput() * FourBarMotor1.getBusVoltage();
        inputs.FourBarCurrentAmps = new double[] {FourBarMotor1.getOutputCurrent()};
        inputs.FourBarTarget = FourBarPIDController.getSetpoint();

        // inputs.FourBarPosition2 = Units.rotationsToRadians(FourBarEncoder2.getPosition());
        // inputs.FourBarVelocity2 = Units.rotationsPerMinuteToRadiansPerSecond(FourBarEncoder2.getVelocity());
        // inputs.FourBarAppliedVolts2 = FourBarMotor2.getAppliedOutput() * FourBarMotor2.getBusVoltage();
        // inputs.FourBarCurrentAmps2 = new double[] {FourBarMotor2.getOutputCurrent()};
    }

    public void setFourBarAngle(double angle) {
        angle = MathUtil.clamp(angle, FourBarConstants.fourBarOut, FourBarConstants.fourBarHome);

        double G = FourBarFeedforward1.calculate(FourBarEncoder1.getPosition(), FourBarEncoder1.getVelocity());
        double PID = FourBarPIDController.calculate(FourBarEncoder1.getPosition(), angle);

        // FourBarMotor1.set(PID + G);

        BuiltInPID.setReference(angle, ControlType.kPosition);

        // LightningShuffleboard.setDouble("four bar", "error", angle - FourBarEncoder1.getPosition());
        // LightningShuffleboard.setDouble("four bar", "PID", PID);
    }

    public double getFourBarAngle() {
        return FourBarEncoder1.getPosition();
    }

    public void setPIDGGains(double kP, double kI, double kD, double kG) {
        FourBarPIDController.setP(kP);
        FourBarPIDController.setI(kI);
        FourBarPIDController.setD(kD);
        FourBarFeedforward1 = new ArmFeedforward(FourBarGains.kS, kG, FourBarGains.kV, FourBarGains.kA);
    }

    public boolean onTarget() {
        return FourBarPIDController.atSetpoint();
    }
}
