// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.FourBar;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.Subsystems.FourBar.FourBarIO.FourBarIOInputs;

public class FourBarIOSparkMax implements FourBarIO {
    /** Creates a new FourBarIOSparkMax. */
    private CANSparkMax FourBarMotor1;

    private CANSparkMax FourBarMotor2;
    private RelativeEncoder FourBarEncoder1;
    private RelativeEncoder FourBarEncoder2;

    private final SparkPIDController FourBarPIDController1;

    public FourBarIOSparkMax() {
        
        FourBarMotor1 = new CANSparkMax(10, CANSparkMax.MotorType.kBrushless);
        FourBarMotor2 = new CANSparkMax(11, CANSparkMax.MotorType.kBrushless);

        FourBarMotor1.setInverted(false);
        FourBarMotor2.setInverted(true);

        FourBarEncoder1 = FourBarMotor1.getEncoder();
        FourBarEncoder2 = FourBarMotor2.getEncoder();

        FourBarMotor1.setSmartCurrentLimit(8);
        FourBarMotor2.setSmartCurrentLimit(8);

        FourBarEncoder1.setPosition(0);
        FourBarEncoder2.setPosition(0);

        FourBarMotor2.follow(FourBarMotor1);

        FourBarPIDController1 = FourBarMotor1.getPIDController();
    }

    @Override
    public void updateInputs(FourBarIOInputs inputs) {
        inputs.FourBarPosition1 = Units.rotationsToRadians(FourBarEncoder1.getPosition());
        inputs.FourBarVelocity1 = Units.rotationsPerMinuteToRadiansPerSecond(FourBarEncoder1.getVelocity());
        inputs.FourBarAppliedVolts1 = FourBarMotor1.getAppliedOutput() * FourBarMotor1.getBusVoltage();
        inputs.FourBarCurrentAmps1 = new double[] {FourBarMotor1.getOutputCurrent()};

        inputs.FourBarPosition2 = Units.rotationsToRadians(FourBarEncoder2.getPosition());
        inputs.FourBarVelocity2 = Units.rotationsPerMinuteToRadiansPerSecond(FourBarEncoder2.getVelocity());
        inputs.FourBarAppliedVolts2 = FourBarMotor2.getAppliedOutput() * FourBarMotor2.getBusVoltage();
        inputs.FourBarCurrentAmps2 = new double[] {FourBarMotor2.getOutputCurrent()};
    }

    public void setFourBarAngle(double angle) {
        FourBarPIDController1.setReference(angle, CANSparkMax.ControlType.kPosition);

    }

    public double getFourBarAngle() {
        return FourBarEncoder1.getPosition();
    }
}
