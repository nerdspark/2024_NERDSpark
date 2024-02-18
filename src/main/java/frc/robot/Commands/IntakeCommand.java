// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake.Intake;
import java.util.function.Supplier;

/** An example command that uses an example subsystem. */
public class IntakeCommand extends Command {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake Intake;

    private Supplier<Double> power;

    public enum IntakeMode {
        FORCEINTAKE,
        FULLINTAKE,
        SOFTINTAKE
    }

    private IntakeMode mode;
    private boolean isIndexing = false;
    private double referencePosition = 0;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public IntakeCommand(Intake Intake, Supplier<Double> power, IntakeMode mode) {
        this.Intake = Intake;
        this.power = power;
        this.mode = mode;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        switch (mode) {
            case FORCEINTAKE:
                 Intake.setIntakePower(power.get());

                break;
            
            case FULLINTAKE: 

                if(!Intake.getBeamBreak()) { 
                    
                    Intake.setIntakePower(power.get()); 
                    isIndexing = false;

                } else if(!isIndexing) { 

                    referencePosition = Intake.getIntakePosition();
                    isIndexing = true;

                }

                if(isIndexing) {

                    if(Math.abs(Intake.getIntakePosition()-referencePosition) > Constants.indexDistance)  {
                        Intake.setIntakePower(0.1);
                    } else {
                        Intake.setIntakePower(0);
                    }
                } 
                
                break;
        
            case SOFTINTAKE:
                if(!Intake.getBeamBreak()) { Intake.setIntakePower(power.get()); }
                else { Intake.setIntakePower(0);}

                break;

            default:
                break;
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
