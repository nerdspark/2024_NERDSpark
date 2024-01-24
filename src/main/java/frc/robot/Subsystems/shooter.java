// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    /** Creates a new shooter. */
    private final CANSparkMax shooterMotor1;

    private final CANSparkMax shooterMotor2;

    public Shooter() {
        shooterMotor1 = new CANSparkMax(Constants.shooterMotor1ID, CANSparkMax.MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(Constants.shooterMotor2ID, CANSparkMax.MotorType.kBrushless);
    }

    public void setSpeed(double speed1, double speed2) {
        shooterMotor1.set(speed1);
        shooterMotor2.set(speed2);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
