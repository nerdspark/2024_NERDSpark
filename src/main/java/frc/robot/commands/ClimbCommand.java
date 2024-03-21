// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.climb.Climb;
import java.util.function.Supplier;

public class ClimbCommand extends Command {
    private final Climb Climb;
    private final boolean isOut;
    private final boolean winchOut;
    /** Creates a new ClimbCommand. */
    public ClimbCommand(Climb Climb, Supplier<Boolean> isOut, Supplier<Boolean> winchOut) {
        this.Climb = Climb;
        this.isOut = isOut.get();
        this.winchOut = winchOut.get();
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (isOut) {
            Climb.setServoPosition(ClimbConstants.servoOutPosition);
        } else {
            Climb.setServoPosition(0);
        }
        if (winchOut) {
            Climb.setClimbPosition(ClimbConstants.winchPos);
        } else {
            Climb.setClimbPosition(0);
        }
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
