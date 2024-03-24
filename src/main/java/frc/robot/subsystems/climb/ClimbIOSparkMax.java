// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.ClimbConstants;

public class ClimbIOSparkMax implements ClimbIO {
    /** Creates a new ClimbIOSparkMax. */
    private TalonFX climbMotor;

    private Servo grapplingServo;
    private double servoSetpoint = ClimbConstants.servoInPos;

    public ClimbIOSparkMax() {
        grapplingServo = new Servo(ClimbConstants.servoPort);
        climbMotor = new TalonFX(ClimbConstants.winchPort);
        climbMotor.setPosition(0);
        climbMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    @SuppressWarnings("static-access")
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.climbPosition = climbMotor.getPosition().getValue();
        inputs.climbVelocity = climbMotor.getVelocity().getValue();
        inputs.climbAppliedVolts = climbMotor.getMotorVoltage().getValue();
        inputs.climbCurrentAmps = new double[] {climbMotor.getStatorCurrent().getValue()};
    }

    public void setClimbMotorPower(double climbPower) {
        climbMotor.set(climbPower);
    }

    public double getClimbMotorPosition() {
        return climbMotor.getPosition().getValue();
    }

    public void setServoPosition(double angle) {
        grapplingServo.setAngle(angle);
        servoSetpoint = angle;
    }

    public boolean getServoOut() {
        return servoSetpoint == ClimbConstants.servoOutPos;
        // return Math.abs(grapplingServo.getAngle() - ClimbConstants.servoOutPos) < ClimbConstants.servoOutTolerance;
    }

    public void setClimbPosition(double position) {
        climbMotor.setControl(new PositionTorqueCurrentFOC(position));
    }
}
