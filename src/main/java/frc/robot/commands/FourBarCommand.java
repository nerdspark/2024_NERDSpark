// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.fourBar.FourBar;
import java.util.function.Supplier;

public class FourBarCommand extends Command {

    private final FourBar FourBar;

    private Supplier<Double> angle;

    /** Creates a new ShooterCommand. */
    public FourBarCommand(FourBar FourBar, Supplier<Double> angle) {
        this.FourBar = FourBar;
        this.angle = angle;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(FourBar);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        FourBar.setFourBarAngle(angle.get());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
