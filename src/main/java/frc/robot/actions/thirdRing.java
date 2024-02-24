package frc.robot.actions;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FourBarCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.fourBar.FourBar;
import frc.robot.subsystems.intake.Intake;

public class thirdRing extends SequentialCommandGroup {

    public thirdRing(FourBar fourBar, Intake intake) {

        addCommands(
                new ParallelCommandGroup(
                        new FourBarCommand(fourBar, () -> 1.0),
                        new IntakeCommand(intake, () -> 1.0, IntakeCommand.IntakeMode.SOFTINTAKE)),
                new FourBarCommand(fourBar, () -> 1.0),
                new IntakeCommand(intake, () -> 1.0, IntakeCommand.IntakeMode.FORCEINTAKE));
    }
}
