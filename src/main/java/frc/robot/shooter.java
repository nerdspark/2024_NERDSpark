// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooter extends SubsystemBase {
    /** Creates a new shooter. */
    private final CANSparkMax shooterMotor1;

    private final CANSparkMax shooterMotor2;
    private final CANSparkMax angleMotor;
    private final SparkPIDController angleMotorPIDController;
    private final RelativeEncoder angleMotorEncoder;

    public shooter() {
        shooterMotor1 = new CANSparkMax(Constants.shooterMotor1ID, CANSparkMax.MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(Constants.shooterMotor2ID, CANSparkMax.MotorType.kBrushless);
        angleMotor = new CANSparkMax(Constants.anglemotorID, CANSparkMax.MotorType.kBrushless);
        angleMotorPIDController = angleMotor.getPIDController();
        angleMotorEncoder = angleMotor.getEncoder();
    }

    public void setSpeed(double speed1, double speed2) {
        shooterMotor1.set(speed1);
        shooterMotor2.set(speed2);
    }

    public void setAngle(double angle) {
        angleMotorPIDController.setReference(angle, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
