// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
// import frc.robot.Subsystems.climb.climbIO.climbIOInputs;

public class ClimbIOSparkMax implements ClimbIO {
    /** Creates a new ClimbIOSparkMax. */
    private CANSparkMax climbMotor1;

    private CANSparkMax climbMotor2;
    private RelativeEncoder climbEncoder1;
    private RelativeEncoder climbEncoder2;

    public ClimbIOSparkMax() {
        climbMotor1 = new CANSparkMax(0, CANSparkMax.MotorType.kBrushless);
        climbMotor2 = new CANSparkMax(0, CANSparkMax.MotorType.kBrushless);

        climbEncoder1 = climbMotor1.getEncoder();
        climbEncoder2 = climbMotor2.getEncoder();

        climbEncoder1.setPosition(0);
        climbEncoder2.setPosition(0);

        climbMotor2.follow(climbMotor1);
    }

    @Override
    @SuppressWarnings("static-access")
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.climbPosition1 = Units.rotationsToRadians(climbEncoder1.getPosition());
        inputs.climbVelocity1 = Units.rotationsPerMinuteToRadiansPerSecond(climbEncoder1.getVelocity());
        inputs.climbAppliedVolts1 = climbMotor1.getAppliedOutput() * climbMotor1.getBusVoltage();
        inputs.climbCurrentAmps1 = new double[] {climbMotor1.getOutputCurrent()};

        inputs.climbPosition2 = Units.rotationsToRadians(climbEncoder2.getPosition());
        inputs.climbVelocity2 = Units.rotationsPerMinuteToRadiansPerSecond(climbEncoder2.getVelocity());
        inputs.climbAppliedVolts2 = climbMotor2.getAppliedOutput() * climbMotor2.getBusVoltage();
        inputs.climbCurrentAmps2 = new double[] {climbMotor2.getOutputCurrent()};
    }

    public void setClimbMotorPower(double climbPower) {
        climbMotor1.set(climbPower);
    }

    public double getClimbMotorPosition() {
        return climbEncoder1.getPosition();
    }
}
