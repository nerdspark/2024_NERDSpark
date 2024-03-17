// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.Constants;

public class GripperIndexCommand extends Command {
  private final Arm arm;
  double startPoint = 0.0;
  // private final Supplier<double> power
  /** Creates a new GripperIndexCommand. */
  public GripperIndexCommand(Arm arm) {
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPoint = arm.getGripperPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setGripperPower(Constants.ArmConstants.indexPowerGripper);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setGripperPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(arm.getGripperPosition() - startPoint) 
    > Constants.ArmConstants.indexDistGripper;
  }
}
