// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.Supplier;

public class ShooterCommand extends Command {

    private final Shooter shooter;

    private Supplier<Double> speed1;
    private Supplier<Double> speed2;

    /** Creates a new ShooterCommand.  */
    public ShooterCommand(Shooter shooter, Supplier<Double> speed1, Supplier<Double> speed2) {
        this.shooter = shooter;
        this.speed1 = speed1;
        this.speed2 = speed2;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.setSpeed(speed1.get(), speed2.get());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.setSpeed(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
