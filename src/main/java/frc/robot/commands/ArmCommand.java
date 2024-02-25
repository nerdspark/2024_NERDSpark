// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import java.util.function.Supplier;

public class ArmCommand extends Command {
    private Arm arm;
    private Supplier<Translation2d> position;
    private Supplier<Double> wrist;
    private Supplier<Boolean> inBend;
    /** Creates a new ArmCommand. */
    public ArmCommand(Arm arm, Supplier<Translation2d> position, Supplier<Double> wrist, Supplier<Boolean> inBend) {
        this.arm = arm;
        this.position = position;
        this.wrist = wrist;
        this.inBend = inBend;
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        arm.getArmPosition();
        arm.setArmPosition(position.get(), inBend.get(), wrist.get());
        // arm.setWristPosition(wrist.get());
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
