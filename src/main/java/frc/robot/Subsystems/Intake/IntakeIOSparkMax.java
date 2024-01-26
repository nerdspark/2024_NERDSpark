// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.Subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

/**
 * This drive implementation is for Spark Maxes driving brushed motors (e.g. CIMS) with no encoders.
 * For the Spark Flex in docked mode, replace all instances of "CANSparkMax" with "CANSparkFlex".
 */
public class IntakeIOSparkMax implements IntakeIO {
    private CANSparkMax intakeMotor1;
    private CANSparkMax intakeMotor2;
    private RelativeEncoder intakeEncoder1;
    private RelativeEncoder intakeEncoder2;

    public IntakeIOSparkMax() {
        intakeMotor1 = new CANSparkMax(10, CANSparkMax.MotorType.kBrushless);
        intakeMotor2 = new CANSparkMax(11, CANSparkMax.MotorType.kBrushless);

        intakeEncoder1 = intakeMotor1.getEncoder();
        intakeEncoder2 = intakeMotor2.getEncoder();

        intakeMotor1.setSmartCurrentLimit(8);
        intakeMotor2.setSmartCurrentLimit(8);

        intakeEncoder1.setPosition(0);
        intakeEncoder2.setPosition(0);

        intakeMotor2.follow(intakeMotor1);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakePosition1 = Units.rotationsToRadians(intakeEncoder1.getPosition());
        inputs.intakeVelocity1 = Units.rotationsPerMinuteToRadiansPerSecond(intakeEncoder1.getVelocity());
        inputs.intakeAppliedVolts1 = intakeMotor1.getAppliedOutput() * intakeMotor1.getBusVoltage();
        inputs.intakeCurrentAmps1 = new double[] {intakeMotor1.getOutputCurrent()};

        inputs.intakePosition2 = Units.rotationsToRadians(intakeEncoder2.getPosition());
        inputs.intakeVelocity2 = Units.rotationsPerMinuteToRadiansPerSecond(intakeEncoder2.getVelocity());
        inputs.intakeAppliedVolts2 = intakeMotor2.getAppliedOutput() * intakeMotor2.getBusVoltage();
        inputs.intakeCurrentAmps2 = new double[] {intakeMotor2.getOutputCurrent()};
    }

    public void setIntakePower(double intakePower) {
        intakeMotor1.set(intakePower);
    }

    public double getIntakePower() {
        return intakeMotor1.get();
    }
}
