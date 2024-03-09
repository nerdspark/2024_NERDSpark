// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.fourBar;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.FourBarConstants;
import frc.robot.Constants.FourBarGains;

public class FourBarIOSparkMax implements FourBarIO {
    /** Creates a new FourBarIOSparkMax. */
    private CANSparkMax FourBarMotor1;

    private CANSparkMax FourBarMotor2;
    private RelativeEncoder FourBarEncoder1;

    private ArmFeedforward fourBarFeedforward1;
    private PIDController FourBarPIDController1;

    public FourBarIOSparkMax() {

        FourBarMotor1 = new CANSparkMax(Constants.fourBarLeftID, CANSparkMax.MotorType.kBrushless);
        FourBarMotor2 = new CANSparkMax(Constants.fourBarRightID, CANSparkMax.MotorType.kBrushless);

        FourBarMotor1.setInverted(true);
        FourBarMotor2.setInverted(false);

        FourBarEncoder1 = FourBarMotor1.getEncoder();

        FourBarMotor1.setSmartCurrentLimit(FourBarConstants.currentLimit);
        FourBarMotor2.setSmartCurrentLimit(FourBarConstants.currentLimit);

        FourBarMotor1.setClosedLoopRampRate(FourBarConstants.closedLoopRampRate);
        FourBarMotor1.setClosedLoopRampRate(FourBarConstants.closedLoopRampRate);
        FourBarMotor1.setOpenLoopRampRate(FourBarConstants.openLoopRampRate);
        FourBarMotor1.setOpenLoopRampRate(FourBarConstants.openLoopRampRate);

        FourBarEncoder1.setPositionConversionFactor(FourBarConstants.positionConversionFactor);
        FourBarEncoder1.setVelocityConversionFactor(FourBarConstants.positionConversionFactor);
        FourBarEncoder1.setPosition(FourBarConstants.resetPosition);

        FourBarMotor2.follow(FourBarMotor1, true);

        // FourBarPIDController1 = FourBarMotor1.getPIDController();
        FourBarPIDController1.setP(FourBarGains.kP);
        FourBarPIDController1.setI(FourBarGains.kI);
        FourBarPIDController1.setD(FourBarGains.kD);
        FourBarPIDController1.setIZone(FourBarGains.kIZone);

        fourBarFeedforward1 = new ArmFeedforward(FourBarGains.kS, FourBarGains.kG, FourBarGains.kV, FourBarGains.kA);
        // fourBarFeedforward2 = new ArmFeedforward(Constants.FourBarGains.kS, Constants.FourBarGains.kG,
        // Constants.FourBarGains.kV, Constants.FourBarGains.kA);
    }

    @Override
    public void updateInputs(FourBarIOInputs inputs) {
        inputs.FourBarPosition1 = Units.rotationsToRadians(FourBarEncoder1.getPosition());
        inputs.FourBarVelocity1 = Units.rotationsPerMinuteToRadiansPerSecond(FourBarEncoder1.getVelocity());
        inputs.FourBarAppliedVolts1 = FourBarMotor1.getAppliedOutput() * FourBarMotor1.getBusVoltage();
        inputs.FourBarCurrentAmps1 = new double[] {FourBarMotor1.getOutputCurrent()};

        // inputs.FourBarPosition2 = Units.rotationsToRadians(FourBarEncoder2.getPosition());
        // inputs.FourBarVelocity2 = Units.rotationsPerMinuteToRadiansPerSecond(FourBarEncoder2.getVelocity());
        // inputs.FourBarAppliedVolts2 = FourBarMotor2.getAppliedOutput() * FourBarMotor2.getBusVoltage();
        // inputs.FourBarCurrentAmps2 = new double[] {FourBarMotor2.getOutputCurrent()};
    }

    public void setFourBarAngle(double angle) {

        FourBarMotor1.set(FourBarPIDController1.calculate(FourBarEncoder1.getPosition(), angle)
                + fourBarFeedforward1.calculate(FourBarEncoder1.getPosition(), FourBarEncoder1.getVelocity()));

        SmartDashboard.putNumber("fourbar Error", angle - FourBarEncoder1.getPosition());
    }

    public double getFourBarAngle() {
        return FourBarEncoder1.getPosition();
    }
}
