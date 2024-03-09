// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.fourBar;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FourBarGains;

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

        setFourBarAngle(LightningShuffleboard.getDouble("four bar", "target", 0.0));

        LightningShuffleboard.setDouble("four bar", "position", getFourBarAngle());

    }

    public void setFourBarAngle(double angle) {
        io.setFourBarAngle(angle);
    }

    public double getFourBarAngle() {
        return io.getFourBarAngle();
    }
}
