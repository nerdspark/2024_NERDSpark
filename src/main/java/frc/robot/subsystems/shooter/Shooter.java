// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    /** Creates a new shooter. */
    private final ShooterIO io;

    private final ShooterIoInputsAutoLogged inputs = new ShooterIoInputsAutoLogged();

    public Shooter(ShooterIO shooterIO) {
        this.io = shooterIO;
    }

    public void setSpeed(double speed1, double speed2) {
        io.setSpeed(speed1, speed2);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("shooter", inputs);
        // This method will be called once per scheduler run
    }

    public void stop() {
        io.stop();
    }
}
