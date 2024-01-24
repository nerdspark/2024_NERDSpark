// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FourBar extends SubsystemBase {
    /** Creates a new FourBar. */
    private final CANSparkMax angleMotor;

    private final SparkPIDController angleMotorPIDController;
    private final RelativeEncoder angleMotorEncoder;

    public FourBar() {
        angleMotor = new CANSparkMax(Constants.anglemotorID, CANSparkMax.MotorType.kBrushless);
        angleMotorPIDController = angleMotor.getPIDController();
        angleMotorEncoder = angleMotor.getEncoder();
    }

    public void setAngle(double angle) {
        angleMotorPIDController.setReference(angle, CANSparkMax.ControlType.kPosition);
    }

    public double getAngle() {
        return angleMotorEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
