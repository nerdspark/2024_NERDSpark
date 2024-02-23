package frc.robot.commands;

import static edu.wpi.first.math.geometry.Rotation2d.fromRadians;
import static edu.wpi.first.units.Units.Meters;
import static java.lang.Math.PI;
import static java.lang.Math.abs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.*;
import frc.robot.util.AutoAimMath;
import frc.robot.util.FieldConstants;
import frc.robot.subsystems.fourBar.*;

public class AutoAimForAutonCommand extends Command {
  private FourBar fourBarSubsystem;
  private Shooter shooterSubsystem;
  private CommandSwerveDrivetrain drivetrainSubsystem;

//   private final Translation2d speakerPosition = team ? speakerPosition1 : speakerPosition2;

  private Pose2d currentDrivetrainPose;
  private Measure<Distance> currentSpeakerDistance;
  private Rotation2d drivetrainAngle;
  private Rotation2d targetRobotAngle;
  private double currentFourBarAngle;


  private Boolean isFourBarReady;
  private Boolean isDrivetrainReady;

  public AutoAimForAutonCommand(FourBar fourBarSubSystem, Shooter shooterSubsystem,
      CommandSwerveDrivetrain drivetrainSubsystem) {
    this.fourBarSubsystem = fourBarSubSystem;
    this.shooterSubsystem = shooterSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;

    addRequirements(fourBarSubSystem, shooterSubsystem, drivetrainSubsystem);
  }

  private Boolean update() {
    currentDrivetrainPose = drivetrainSubsystem.getState().Pose;
    currentSpeakerDistance = Meters.of(currentDrivetrainPose.getTranslation().getDistance(FieldConstants.Speaker.centerSpeakerOpening.getTranslation()));
    drivetrainAngle = currentDrivetrainPose.getRotation();
    currentFourBarAngle = fourBarSubsystem.getFourBarAngle();
    var targetFourBarAngle = AutoAimMath.getAutoAimCalcFourBar(()->currentDrivetrainPose,FieldConstants.Speaker.centerSpeakerOpening);
    
    targetRobotAngle = currentDrivetrainPose.getRotation().minus(FieldConstants.Speaker.centerSpeakerOpening.getRotation());

    isFourBarReady = abs(currentFourBarAngle - targetFourBarAngle) <= Constants.ShooterConstants.FOURBAR_ANGLE_THRESHOLD;
    isDrivetrainReady = currentSpeakerDistance.lte(Constants.ShooterConstants.MAXIMUM_READYSHOOT_DISTANCE);

    return isFourBarReady && isDrivetrainReady;
  }

  @Override
  public void execute() {
    update();
    shooterSubsystem.setSpeed(Constants.ShooterConstants.SHOOTER_SPEED, Constants.ShooterConstants.SHOOTER_SPEED);

  }

  @Override
  public boolean isFinished() {
    // update returns if the turret is ready for shooting: the turret is correctly
    // angled, the drivetrain is correctly angled, the robot is within the maximum
    // distance to the speaker, and the shooter wheel is ready
    return update();
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setSpeed(0,0);
  }
}
