package frc.robot.actions;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.FourBarCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.fourBar.FourBar;
import frc.robot.subsystems.intake.Intake;
import frc.robot.Constants;

public class backToSafety extends ParallelCommandGroup{

    public backToSafety(
    Intake intake,
    FourBar fourBar){
        new IntakeCommand(intake, () -> 1.0, IntakeCommand.IntakeMode.FULLINTAKE);
        new FourBarCommand(fourBar, () -> Constants.fourBarHome);
    }
}
