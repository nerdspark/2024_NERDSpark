// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.fourBar;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FourBar extends SubsystemBase {
    /** Creates a new FourBar. */
    private final FourBarIO io;

    private final FourBarIOInputsAutoLogged inputs = new FourBarIOInputsAutoLogged();

    public FourBar(FourBarIO FourBarIO) {
        this.io = FourBarIO;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        // This method will be called once per scheduler run

    }

    public void setFourBarAngle(double angle) {
        io.setFourBarAngle(angle);
    }

     public double getFourBarAngle() {
        return io.getFourBarAngle();
    }
}