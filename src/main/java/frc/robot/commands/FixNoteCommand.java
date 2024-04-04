// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import java.util.function.Supplier;

public class FixNoteCommand extends Command {
    private final Intake Intake;
    private boolean smallBeamBreak;
    public final Timer timer = new Timer();
    private Supplier<Double> power;

    /** Creates a new FixNote. */
    public FixNoteCommand(Intake Intake, Supplier<Double> power) {
        this.Intake = Intake;
        this.power = power;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.start();
        timer.reset();
        SmartDashboard.putBoolean("smallBeambreak2", Intake.getSmallBeamBreak());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Intake.getBeamBreak() == true) {
            Intake.setIntakePower(0.2);
            if (Intake.getSmallBeamBreak() == true) {
                timer.reset();
                Intake.setIntakePower(-0.2); // add a time command
                if (timer.hasElapsed(0.2) == true) {
                    Intake.setIntakePower(0.1);
                    if (Intake.getSmallBeamBreak() == true) {
                        Intake.setIntakePower(0);
                    }
                }
            }
        }
    }
    ;
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
