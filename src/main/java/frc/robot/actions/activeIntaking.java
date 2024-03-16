package frc.robot.actions;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.FourBarConstants;
import frc.robot.commands.FourBarCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.fourBar.FourBar;
import frc.robot.subsystems.intake.Intake;

public class activeIntaking extends ParallelCommandGroup {

    public activeIntaking(Intake intake, FourBar fourBar) {

        new FourBarCommand(fourBar, () -> FourBarConstants.fourBarOut);
        new IntakeCommand(intake, () -> 1.0, IntakeCommand.IntakeMode.SOFTINTAKE);
    }
}
