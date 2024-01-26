// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.Intake;
import java.util.function.Supplier;

/** An example command that uses an example subsystem. */
public class IntakeCommand extends Command {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake Intake;

    private Supplier<Double> power;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public IntakeCommand(Intake Intake, Supplier<Double> power) {
        this.Intake = Intake;
        this.power = power;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Intake.setIntakePower(power.get());
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
