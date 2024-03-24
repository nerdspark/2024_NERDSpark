// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
    /** Creates a new Climb. */
    private final ClimbIO io;

    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    public Climb(ClimbIO climbIO) {
        this.io = climbIO;
    }

    public void setClimbMotorPower(double climbPower) {
        io.setClimbMotorPower(climbPower);
    }

    public double getClimbMotorPosition() {
        return io.getClimbMotorPosition();
    }

    public void setServoPosition(double angle) {
        io.setServoPosition(angle);
    }

    public void setClimbPosition(double position) {
        io.setClimbPosition(ClimbConstants.winchPos);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("climb", inputs);
    }
}
