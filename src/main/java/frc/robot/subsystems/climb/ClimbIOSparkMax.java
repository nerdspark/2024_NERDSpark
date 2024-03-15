// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
// import frc.robot.Subsystems.climb.climbIO.climbIOInputs;

public class ClimbIOSparkMax implements ClimbIO {
    /** Creates a new ClimbIOSparkMax. */
    private CANSparkMax climbMotor;

    private RelativeEncoder climbEncoder;

    public ClimbIOSparkMax() {
        climbMotor = new CANSparkMax(0, CANSparkMax.MotorType.kBrushless);

        climbEncoder = climbMotor.getEncoder();

        climbEncoder.setPosition(0);
    }

    @Override
    @SuppressWarnings("static-access")
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.climbPosition = climbEncoder.getPosition();
        inputs.climbVelocity = climbEncoder.getVelocity();
        inputs.climbAppliedVolts = climbMotor.getAppliedOutput() * climbMotor.getBusVoltage();
        inputs.climbCurrentAmps = new double[] {climbMotor.getOutputCurrent()};
    }

    public void setClimbMotorPower(double climbPower) {
        climbMotor.set(climbPower);
    }

    public double getClimbMotorPosition() {
        return climbEncoder.getPosition();
    }
}
