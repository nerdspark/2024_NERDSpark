// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import java.util.function.Supplier;

/** An example command that uses an example subsystem.  */
public class IntakeCommand extends Command {

    public final Timer timer = new Timer();

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake Intake;

    private Supplier<Double> power;

    public enum IntakeMode {
        FORCEINTAKE,
        FORCEINTAKESHOOT,
        FULLINTAKE,
        SOFTINTAKE
    }

    private IntakeMode mode;
    private boolean isIndexing = false;
    private double referencePosition = 0;

    public IntakeCommand(Intake Intake, Supplier<Double> power, IntakeMode mode) {
        this.Intake = Intake;
        this.power = power;
        this.mode = mode;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putBoolean("gotNote", Intake.getBeamBreak());

        switch (mode) {
            case FORCEINTAKE:
                Intake.setIntakePower(power.get());

                break;

            case FORCEINTAKESHOOT:
                Intake.setIntakePower(power.get());

                break;

            case FULLINTAKE:
                if (!Intake.getBeamBreak()) {

                    Intake.setIntakePower(power.get());
                    isIndexing = false;

                } else if (!isIndexing) {

                    referencePosition = Intake.getIntakePosition();
                    isIndexing = true;
                }

                // if (isIndexing) {

                //     if (Math.abs(Intake.getIntakePosition() - referencePosition) > Constants.indexDistance) {
                //         Intake.setIntakePower(0.1);
                //     } else {
                //         Intake.setIntakePower(0);
                //     }
                // }

                break;

            case SOFTINTAKE:
                if (!Intake.getBeamBreak()) {
                    Intake.setIntakePower(power.get());

                    // lights.setLightPattern(BlinkinLightsConstants.doesNotHaveNotePattern);
                } else {
                    Intake.setIntakePower(0);
                    // lights.setLightPattern(BlinkinLightsConstants.hasNotePattern);
                }

                break;

            default:
                break;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Intake.setIntakePower(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        switch (mode) {
            case FORCEINTAKE:
                return timer.hasElapsed(0.5);

            case FORCEINTAKESHOOT:
                return timer.hasElapsed(0.5);

            case SOFTINTAKE:
                return Intake.getBeamBreak();

            case FULLINTAKE:
                return Intake.getBeamBreak();

            default:
                return false;
        }
    }
}
