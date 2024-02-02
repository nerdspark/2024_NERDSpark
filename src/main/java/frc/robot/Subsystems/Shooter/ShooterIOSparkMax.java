// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Subsystems.Shooter.ShooterIO.ShooterIoInputs;

public class ShooterIOSparkMax implements ShooterIO {
    /** Creates a new ShooterIOSparkMax. */

    /** Creates a new shooter. */
    private final CANSparkMax shooterMotor1;

    private final CANSparkMax shooterMotor2;

    private final RelativeEncoder shooterEncoder1;

    private final RelativeEncoder shooterEncoder2;

    public ShooterIOSparkMax() {
        shooterMotor1 = new CANSparkMax(Constants.shooterMotor1ID, CANSparkMax.MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(Constants.shooterMotor2ID, CANSparkMax.MotorType.kBrushless);
        shooterEncoder1 = shooterMotor1.getEncoder();
        shooterEncoder2 = shooterMotor2.getEncoder();
    }

    @Override
    public void updateInputs(ShooterIoInputs inputs) {
        inputs.shooterPosition1 = Units.rotationsToRadians(shooterEncoder1.getPosition());
        inputs.shooterVelocity1 = Units.rotationsPerMinuteToRadiansPerSecond(shooterEncoder1.getVelocity());
        inputs.shooterAppliedVolts1 = shooterMotor1.getAppliedOutput() * shooterMotor1.getBusVoltage();
        inputs.shooterCurrentAmps1 = new double[] {shooterMotor1.getOutputCurrent()};

        inputs.shooterPosition2 = Units.rotationsToRadians(shooterEncoder2.getPosition());
        inputs.shooterVelocity2 = Units.rotationsPerMinuteToRadiansPerSecond(shooterEncoder2.getVelocity());
        inputs.shooterAppliedVolts2 = shooterMotor2.getAppliedOutput() * shooterMotor2.getBusVoltage();
        inputs.shooterCurrentAmps2 = new double[] {shooterMotor2.getOutputCurrent()};
    }

    public void setSpeed(double speed1, double speed2) {
        shooterMotor1.set(speed1);
        shooterMotor2.set(speed2);
    }
}