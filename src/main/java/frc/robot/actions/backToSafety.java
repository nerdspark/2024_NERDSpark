package frc.robot.actions;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.FourBarConstants;
import frc.robot.commands.FourBarCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.fourBar.FourBar;
import frc.robot.subsystems.intake.Intake;

public class backToSafety extends ParallelCommandGroup {

    public backToSafety(Intake intake, FourBar fourBar) {
        new IntakeCommand(intake, () -> 0.0, IntakeCommand.IntakeMode.FORCEINTAKE);
        new FourBarCommand(fourBar, () -> FourBarConstants.fourBarHome);
    }
}
