// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.Supplier;

/** An example command that uses an example subsystem. */
public class DriveToPoseCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final CommandSwerveDrivetrain drivetrainSubsystem;

    private final Supplier<Pose2d> targetPoseSupplier;

    private final Supplier<Pose2d> currentPoseProvider;
    // private  final Pose2d goalPose;

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS =
            new TrapezoidProfile.Constraints(VisionConstants.MAX_VELOCITY, VisionConstants.MAX_ACCELARATION);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
            new TrapezoidProfile.Constraints(VisionConstants.MAX_VELOCITY, VisionConstants.MAX_ACCELARATION);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = new TrapezoidProfile.Constraints(
            VisionConstants.MAX_VELOCITY_ROTATION, VisionConstants.MAX_ACCELARATION_ROTATION);
    private final ProfiledPIDController xController = new ProfiledPIDController(
            VisionConstants.kPXController, VisionConstants.kIXController, VisionConstants.kIXController, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(
            VisionConstants.kPYController, VisionConstants.kIYController, VisionConstants.kDYController, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(
            VisionConstants.kPThetaController,
            VisionConstants.kIThetaController,
            VisionConstants.kDThetaController,
            OMEGA_CONSTRATINTS);

    private final SwerveRequest.ApplyChassisSpeeds driveToPoseRequest = new SwerveRequest.ApplyChassisSpeeds();

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public DriveToPoseCommand(
            CommandSwerveDrivetrain drivetrainSubsystem,
            Supplier<Pose2d> poseProvider,
            Supplier<Pose2d> goalPoseSupplier) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.currentPoseProvider = poseProvider;
        this.targetPoseSupplier = goalPoseSupplier;

        xController.setTolerance(VisionConstants.TRANSLATION_TOLERANCE_X);
        yController.setTolerance(VisionConstants.TRANSLATION_TOLERANCE_Y);
        omegaController.setTolerance(VisionConstants.ROTATION_TOLERANCE);
        omegaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drivetrainSubsystem);
    }

    /** Drives to the specified pose when passed a target pose */
    public DriveToPoseCommand(CommandSwerveDrivetrain drivetrainSubsystem, Supplier<Pose2d> poseProvider, Pose2d pose) {
        this(drivetrainSubsystem, poseProvider, () -> pose);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        SmartDashboard.putString("DriveToPoseCommand", "Initialize");

        var robotPose = currentPoseProvider.get();

        omegaController.reset(
                robotPose.getRotation().getRadians(),
                drivetrainSubsystem.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond);
        xController.reset(robotPose.getX(), drivetrainSubsystem.getCurrentRobotChassisSpeeds().vxMetersPerSecond);
        yController.reset(robotPose.getY(), drivetrainSubsystem.getCurrentRobotChassisSpeeds().vyMetersPerSecond);

        SmartDashboard.putNumber(
                "YawVelocity", drivetrainSubsystem.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond);
        SmartDashboard.putNumber(
                "FieldVelocityX", drivetrainSubsystem.getCurrentRobotChassisSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber(
                "FieldVelocityY", drivetrainSubsystem.getCurrentRobotChassisSpeeds().vyMetersPerSecond);

        omegaController.setGoal(targetPoseSupplier.get().getRotation().getRadians());
        xController.setGoal(targetPoseSupplier.get().getX());
        yController.setGoal(targetPoseSupplier.get().getY());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // SmartDashboard.putString("DriveToPoseCommand", "Execute");

        var robotPose = currentPoseProvider.get();
        // SmartDashboard.putNumber("DriveToPoseCommand robotPose.X", robotPose.getX());
        // SmartDashboard.putNumber("DriveToPoseCommand robotPose.Y", robotPose.getY());
        // SmartDashboard.putNumber(
        //         "DriveToPoseCommand robotPose.Angle", robotPose.getRotation().getRadians());

        // SmartDashboard.putNumber(
        //         "DriveToPoseCommand goalPose.X", targetPoseSupplier.get().getX());
        // SmartDashboard.putNumber(
        //         "DriveToPoseCommand goalPose.Y", targetPoseSupplier.get().getY());
        // SmartDashboard.putNumber(
        //         "DriveToPoseCommand goalPose.Angle",
        //         targetPoseSupplier.get().getRotation().getRadians());

        var xSpeed = xController.calculate(robotPose.getX());
        if (xController.atGoal()) {
            xSpeed = 0;
        }

        var ySpeed = yController.calculate(robotPose.getY());
        if (yController.atGoal()) {
            ySpeed = 0;
        }

        var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());
        if (omegaController.atGoal()) {
            omegaSpeed = 0;
        }

        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, omegaSpeed, drivetrainSubsystem.getRotation3d().toRotation2d());

        drivetrainSubsystem.applyRequest(() -> driveToPoseRequest.withSpeeds(chassisSpeeds));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ChassisSpeeds zeroChassisSpeeds = new ChassisSpeeds();
        drivetrainSubsystem.applyRequest(() -> driveToPoseRequest.withSpeeds(zeroChassisSpeeds));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return xController.atGoal() && yController.atGoal() && omegaController.atGoal();
    }
}
