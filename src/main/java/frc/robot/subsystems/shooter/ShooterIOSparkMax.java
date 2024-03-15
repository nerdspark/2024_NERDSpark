// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.RobotMap;

public class ShooterIOSparkMax implements ShooterIO {
    /** Creates a new ShooterIOSparkMax. */

    /** Creates a new shooter. */
    private final CANSparkMax shooterMotor1;

    private final CANSparkMax shooterMotor2;

    private final RelativeEncoder shooterEncoder1;

    private final RelativeEncoder shooterEncoder2;

    private final SparkPIDController shooterController1;
    private final SparkPIDController shooterController2;

    public ShooterIOSparkMax() {
        shooterMotor1 = new CANSparkMax(RobotMap.shooterMotor1ID, CANSparkMax.MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(RobotMap.shooterMotor2ID, CANSparkMax.MotorType.kBrushless);
        shooterMotor1.setInverted(false);
        shooterMotor2.setInverted(true);
        shooterEncoder1 = shooterMotor1.getEncoder();
        shooterEncoder2 = shooterMotor2.getEncoder();
        shooterMotor1.setSmartCurrentLimit(60);
        shooterMotor2.setSmartCurrentLimit(60);
        shooterMotor1.setIdleMode(IdleMode.kCoast);
        shooterMotor2.setIdleMode(IdleMode.kCoast);
        shooterMotor1.setClosedLoopRampRate(0.1);
        shooterMotor2.setClosedLoopRampRate(0.1);

        shooterController1 = shooterMotor1.getPIDController();
        shooterController2 = shooterMotor2.getPIDController();

        shooterController1.setP(0.00038);
        shooterController1.setI(2E-7);
        shooterController1.setD(0.0125);
        shooterController1.setIZone(1000);
        shooterController1.setFF(0.000163);
        shooterController2.setP(0.00038);
        shooterController2.setI(2E-7);
        shooterController2.setD(0.0125);
        shooterController2.setIZone(1000);
        shooterController2.setFF(0.000163);
    }

    @Override
    @SuppressWarnings("static-access")
    public void updateInputs(ShooterIoInputs inputs) {
        // inputs.shooterPosition1 = shooterEncoder1.getPosition();
        inputs.shooterVelocity1 = shooterEncoder1.getVelocity();
        // inputs.shooterAppliedVolts1 = shooterMotor1.getAppliedOutput() * shooterMotor1.getBusVoltage();
        // inputs.shooterCurrentAmps1 = new double[] {shooterMotor1.getOutputCurrent()};

        // inputs.shooterPosition2 = shooterEncoder2.getPosition();
        inputs.shooterVelocity2 = shooterEncoder2.getVelocity();
        // inputs.shooterAppliedVolts2 = shooterMotor2.getAppliedOutput() * shooterMotor2.getBusVoltage();
        // inputs.shooterCurrentAmps2 = new double[] {shooterMotor2.getOutputCurrent()};

        inputs.shooterI1 = shooterController1.getIAccum();
        inputs.shooterI2 = shooterController2.getIAccum();
    }

    public void setSpeed(double speed1, double speed2) {
        shooterController1.setReference(speed1, ControlType.kVelocity);
        shooterController2.setReference(speed2, ControlType.kVelocity);
    }

    public void stop() {
        shooterMotor1.set(0);
        shooterMotor2.set(0);
    }
}
