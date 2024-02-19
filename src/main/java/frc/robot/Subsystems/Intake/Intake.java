// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    // private CANSparkMax intakeMotor;
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    /** Creates a new ExampleSubsystem. */
    public Intake(IntakeIO intakeIO) {
        this.io = intakeIO;
        // intakeMotor = new CANSparkMax(Constants.intakeMotorId, CANSparkMax.MotorType.kBrushless);
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */

    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation

    }

    public void setIntakePower(double power) {
        io.setIntakePower(power);
    }

    public boolean getBeamBreak() {
        return io.getBeamBreak();
    }

    public double getIntakePosition() {
        return io.getIntakePosition();
    }
}
