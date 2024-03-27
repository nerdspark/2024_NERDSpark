// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.climb.Climb;
import java.util.function.Supplier;

public class GrapplerCommand extends Command {
    private final Climb Climb;
    // private final boolean servoRelease;
    /** Creates a new ClimbCommand. */
    public GrapplerCommand(Climb Climb) {
        this.Climb = Climb;
        // this.servoRelease = servoRelease.get();
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // if (servoRelease) {
            Climb.setServoPosition(ClimbConstants.servoOutPos);
        // } else {
            // Climb.setServoPosition(ClimbConstants.servoInPos);
        // }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Climb.setServoPosition(ClimbConstants.servoInPos);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
