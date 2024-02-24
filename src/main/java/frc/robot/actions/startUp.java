package frc.robot.actions;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FourBarCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.fourBar.FourBar;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class startUp extends SequentialCommandGroup {

    public startUp(Shooter shooter, FourBar fourBar, Intake intake) {

        addCommands(
                new ParallelCommandGroup(
                        new ShooterCommand(shooter, () -> 1.0, () -> 2.0),
                        new FourBarCommand(fourBar, () -> 1.0)),
                new IntakeCommand(intake, () -> 1.0, IntakeCommand.IntakeMode.FORCEINTAKE));
    }
}
