// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax deployMotor;
    private CANSparkMax intakeMotor;

    private SparkPIDController deployMotorPIDcontroller;
    private RelativeEncoder deployEncoder;

    /** Creates a new ExampleSubsystem. */
    public IntakeSubsystem() {
        deployMotor = new CANSparkMax(Constants.deployMotorId, CANSparkMax.MotorType.kBrushless);
        intakeMotor = new CANSparkMax(Constants.intakeMotorId, CANSparkMax.MotorType.kBrushless);

        deployMotorPIDcontroller = deployMotor.getPIDController();
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

    public void setdeployPower(double pos) {
        deployMotorPIDcontroller.setReference(pos, CANSparkMax.ControlType.kPosition);
    }

    public double getIntakePower() {
        return intakeMotor.get();
    }

    public double getDeployPos() {
        return deployEncoder.getPosition();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation

    }
}
