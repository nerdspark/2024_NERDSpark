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

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants.RobotMap;

/**
 * This drive implementation is for Spark Maxes driving brushed motors (e.g. CIMS) with no encoders.
 * For the Spark Flex in docked mode, replace all instances of "CANSparkMax" with "CANSparkFlex".
 */
public class IntakeIOSparkMax implements IntakeIO {
    private CANSparkMax intakeMotor;

    private RelativeEncoder intakeEncoder;

    private AnalogInput beamBreak;

    public IntakeIOSparkMax() {
        intakeMotor = new CANSparkMax(RobotMap.intakeMotorId, CANSparkMax.MotorType.kBrushless);

        intakeMotor.setInverted(true);

        intakeEncoder = intakeMotor.getEncoder();

        intakeMotor.setSmartCurrentLimit(60);
        intakeMotor.setOpenLoopRampRate(0);

        intakeMotor.setIdleMode(IdleMode.kBrake);

        intakeEncoder.setPosition(0);

        beamBreak = new AnalogInput(0);
    }

    @Override
    @SuppressWarnings("static-access")
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakePosition1 = intakeEncoder.getPosition();
        inputs.intakeVelocity1 = intakeEncoder.getVelocity();
        inputs.intakeAppliedVolts1 = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
        inputs.intakeCurrentAmps1 = new double[] {intakeMotor.getOutputCurrent()};
    }

    public void setIntakePower(double intakePower) {
        intakeMotor.set(intakePower);
    }

    public double getIntakePower() {
        return intakeMotor.get();
    }

    public boolean getBeamBreak() {
        return beamBreak.getVoltage() < 4;
    }

    public double getIntakePosition() {
        return intakeEncoder.getPosition();
    }
}
