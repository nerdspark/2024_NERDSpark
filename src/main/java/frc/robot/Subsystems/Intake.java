// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private CANSparkMax intakeMotor;

    /** Creates a new ExampleSubsystem. */
    public Intake() {
        intakeMotor = new CANSparkMax(Constants.intakeMotorId, CANSparkMax.MotorType.kBrushless);
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
        // This method will be called once per scheduler run

    }

    public void setIntakePower(double intakePower) {
        intakeMotor.set(intakePower);
    }

    public double getIntakePower() {
        return intakeMotor.get();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation

    }
}
