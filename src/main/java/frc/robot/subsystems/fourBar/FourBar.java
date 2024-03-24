// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.fourBar;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FourBarConstants;
import frc.robot.util.LightningShuffleboard;

import org.littletonrobotics.junction.Logger;

public class FourBar extends SubsystemBase {
    /** Creates a new FourBar. */
    private final FourBarIO io;

    private final FourBarIOInputsAutoLogged inputs = new FourBarIOInputsAutoLogged();

    private double targetAngle = FourBarConstants.fourBarHome;

    public FourBar(FourBarIO FourBarIO) {
        this.io = FourBarIO;

        // LightningShuffleboard.setDoubleSupplier("four bar", "position", this::getFourBarAngle);

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("fourBar", inputs);
        // if(LightningShuffleboard.getBool("four bar", "useInput", false)) {
        // setFourBarAngle(LightningShuffleboard.getDouble("four bar", "target", FourBarConstants.fourBarHome));
        // }

        // io.setPIDGGains(LightningShuffleboard.getDouble("four bar", "kP", FourBarGains.kP),
        // LightningShuffleboard.getDouble("four bar", "kI", FourBarGains.kI), LightningShuffleboard.getDouble("four
        // bar", "kD", FourBarGains.kD), LightningShuffleboard.getDouble("four bar", "kG", FourBarGains.kG));

        io.setFourBarAngle(targetAngle);
    }

    public void setFourBarAngle(double angle) {
        targetAngle = angle;
    }

    public boolean onTarget() {
        return io.onTarget();
    }

    public double getFourBarAngle() {
        return io.getFourBarAngle();
    }
}
