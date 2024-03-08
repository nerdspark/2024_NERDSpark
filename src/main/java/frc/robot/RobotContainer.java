// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants.ArmSetPoints;
import frc.robot.Constants.ArmConstants.ClimbSetPoints;
import frc.robot.actions.activeIntaking;
import frc.robot.actions.backToSafety;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmResetCommand;
import frc.robot.commands.FourBarCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeCommand.IntakeMode;
import frc.robot.commands.ShooterCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.fourBar.FourBar;
import frc.robot.subsystems.fourBar.FourBarIO;
import frc.robot.subsystems.fourBar.FourBarIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.AprilTagVisionIOPhotonVision;
import frc.robot.subsystems.vision.PoseEstimatorSubsystem;
import frc.robot.util.AutoAim;
import frc.robot.util.JoystickMap;
import frc.robot.util.RobotConstants;

import java.util.function.Supplier;

public class RobotContainer {
    private double MaxSpeed = 6.0; // 6 meters per second desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    private TunerConstants tunerConstants = RobotConstants.getRobotConstants(RobotIdentity.getIdentity());
    private Intake intake;
    private FourBar fourBar;
    private Shooter shooter;
    private Arm arm;

    private SlewRateLimiter xLimiter = new SlewRateLimiter(8);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(8);
    private SlewRateLimiter zLimiter = new SlewRateLimiter(7);
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandXboxController driver = new CommandXboxController(0); // My joystick
    private final CommandXboxController copilot = new CommandXboxController(1);
   private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

    private final XboxController driverRaw = new XboxController(0);
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
    // driving in open loop
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final SendableChooser<Command> autoChooser;
    private PIDController gyroPid = new PIDController(Constants.gyroP, Constants.gyroI, Constants.gyroD);
    private double targetAngle = 0;
    private final Pigeon2 gyro = new Pigeon2(Constants.pigeonID, "canivore1");
    private double gyroOffset = gyro.getAngle();
    private AprilTagVision aprilTagVision;
    private PoseEstimatorSubsystem poseEstimatorSubSystem;
    //     private NoteVisionSubsystem noteVisionSubsystem =
    //             new NoteVisionSubsystem(Constants.VisionConstants.NOTE_CAMERA_NAME);

    private void configureBindings() {
        // drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        //     new DriveCommand(drivetrain,() -> driver.getLeftX(),() -> driver.getRightX(),() -> driver.getLeftY(),()
        // -> driver.getRightY(),() -> driverRaw.getPOV()));
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
        () -> drive.withVelocityX(
        xLimiter.calculate(-JoystickMap.JoystickPowerCalculate(driver.getRightY())
        * MaxSpeed)) // Drive forward with
        // negative Y (forward)
        .withVelocityY(
        yLimiter.calculate(-JoystickMap.JoystickPowerCalculate(driver.getRightX())
        * MaxSpeed)) // Drive left with negative X (left)
        .withRotationalRate(
        calculateAutoTurn(() -> 0.0)) // Drive counterclockwise with negative X (left)
        ));

        // driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driver.b()
        //         .whileTrue(drivetrain.applyRequest(
        //                 () -> point.withModuleDirection(new Rotation2d(-driver.getRightY(), -driver.getRightX()))));
        driver.start().onTrue(new InstantCommand(() -> resetGyro()));
        // reset the field-centric heading on left bumper press
        // driver.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        drivetrain.registerTelemetry(logger::telemeterize);
        // arm.setDefaultCommand(new ArmCommand(arm, () -> ArmSetPoints.home, () -> ArmSetPoints.homeWrist, () ->
        // false));
        // fourBar.setDefaultCommand(new FourBarCommand(fourBar, () -> Constants.fourBarHome));
    }

    public RobotContainer() {

        switch (Constants.currentMode) {
            case REAL:
                intake = new Intake(new IntakeIOSparkMax()); // Spark Max
                fourBar = new FourBar(new FourBarIOSparkMax());
                shooter = new Shooter(new ShooterIOSparkMax());
                arm = new Arm(new ArmIOSparkMax());
                break;

            default:
                // Replayed robot, disable IO implementations
                intake = new Intake(new IntakeIO() {});
                shooter = new Shooter(new ShooterIO() {});
                fourBar = new FourBar(new FourBarIO() {});
                arm = new Arm(new ArmIO() {});
                // break;
        }

        drivetrain.setRobotIntake(intake);
        drivetrain.getModule(0).getDriveMotor().setInverted(false);
        drivetrain.getModule(1).getDriveMotor().setInverted(true); // FR
        drivetrain.getModule(2).getDriveMotor().setInverted(true); // b
        drivetrain.getModule(3).getDriveMotor().setInverted(true); // b

        NamedCommands.registerCommand(
                "shootSpeed",
                new ShooterCommand(
                        shooter,
                        () -> AutoAim.calculateShooterRPM(
                                () -> drivetrain.getState().Pose,
                                () -> new Translation2d(
                                        drivetrain.getState().speeds.vxMetersPerSecond,
                                        drivetrain.getState().speeds.vyMetersPerSecond)),
                        () -> AutoAim.calculateShooterRPM(
                                () -> drivetrain.getState().Pose,
                                () -> new Translation2d(
                                        drivetrain.getState().speeds.vxMetersPerSecond,
                                        drivetrain.getState().speeds.vyMetersPerSecond))));

        NamedCommands.registerCommand("shootOff", new ShooterCommand(shooter, () -> 0.0, () -> 0.0));

        NamedCommands.registerCommand("fourBarToIntake", new FourBarCommand(fourBar, () -> Constants.fourBarOut));
        NamedCommands.registerCommand("fourBarToHome", new FourBarCommand(fourBar, () -> Constants.fourBarHome));

        NamedCommands.registerCommand(
                "fourBarToShooter",
                new FourBarCommand(
                        fourBar,
                        () -> AutoAim.calculateFourBarPosition(
                                () -> drivetrain.getState().Pose,
                                () -> new Translation2d(
                                        drivetrain.getState().speeds.vxMetersPerSecond,
                                        drivetrain.getState().speeds.vyMetersPerSecond))));

        NamedCommands.registerCommand("fourBarToRCR2", new FourBarCommand(fourBar, () -> Constants.redCenterRing2));
        NamedCommands.registerCommand("fourBarToRCR3", new FourBarCommand(fourBar, () -> Constants.redCenterRing3));
        NamedCommands.registerCommand("fourBarToRCR4", new FourBarCommand(fourBar, () -> Constants.redCenterRing4));

        NamedCommands.registerCommand("fourBarToBCR2", new FourBarCommand(fourBar, () -> Constants.blueCenterRing2));
        NamedCommands.registerCommand("fourBarToBCR3", new FourBarCommand(fourBar, () -> Constants.blueCenterRing3));
        NamedCommands.registerCommand("fourBarToBCR4", new FourBarCommand(fourBar, () -> Constants.blueCenterRing4));

        NamedCommands.registerCommand("fourBarToWSR1", new FourBarCommand(fourBar, () -> Constants.weirdSideRing1));
        NamedCommands.registerCommand("fourBarToWSR2", new FourBarCommand(fourBar, () -> Constants.weirdSideRing2));
        NamedCommands.registerCommand("fourBarToWSR3", new FourBarCommand(fourBar, () -> Constants.weirdSideRing3));
        NamedCommands.registerCommand("fourBarToWSR4", new FourBarCommand(fourBar, () -> Constants.weirdSideRing4));

        NamedCommands.registerCommand(
                "RfourBarToWSR1", new FourBarCommand(fourBar, () -> Constants.red_weirdSideRing1));
        NamedCommands.registerCommand(
                "RfourBarToWSR2", new FourBarCommand(fourBar, () -> Constants.red_weirdSideRing2));
        NamedCommands.registerCommand(
                "RfourBarToWSR3", new FourBarCommand(fourBar, () -> Constants.red_weirdSideRing3));
        NamedCommands.registerCommand(
                "RfourBarToWSR4", new FourBarCommand(fourBar, () -> Constants.red_weirdSideRing4));

        NamedCommands.registerCommand(
                "forcedIntake", new IntakeCommand(intake, () -> 1.0, IntakeCommand.IntakeMode.FORCEINTAKE));

        NamedCommands.registerCommand(
                "forcedIntakeShoot", new IntakeCommand(intake, () -> 1.0, IntakeCommand.IntakeMode.FORCEINTAKESHOOT));

        NamedCommands.registerCommand(
                "forcedIntakeZero", new IntakeCommand(intake, () -> 0.0, IntakeCommand.IntakeMode.FORCEINTAKE));

        NamedCommands.registerCommand(
                "onlySoftIntake", new IntakeCommand(intake, () -> 0.6, IntakeCommand.IntakeMode.SOFTINTAKE));

        NamedCommands.registerCommand("backToSafety", new backToSafety(intake, fourBar));
        NamedCommands.registerCommand("intakingRings", new activeIntaking(intake, fourBar));

        configureBindings();

        configureDashboard();
        autoChooser = AutoBuilder.buildAutoChooser();
        Shuffleboard.getTab("Autonomous").add(autoChooser);
        if (Constants.VisionConstants.USE_VISION == true) {
            if (Constants.VisionConstants.USE_ADV_KIT_VISION == true) {
                aprilTagVision = new AprilTagVision(new AprilTagVisionIOPhotonVision());
                aprilTagVision.setDataInterfaces(drivetrain::addVisionData);
                aprilTagVision.setPoseProvider(drivetrain::getCurrentPose);
            } else {
                poseEstimatorSubSystem = new PoseEstimatorSubsystem(drivetrain);
            }
        }

        // intake button
        driver.leftTrigger()
                .whileTrue(new SequentialCommandGroup(
                        new IntakeCommand(intake, () -> ((driverRaw.getLeftTriggerAxis() - 0.4)), IntakeMode.SOFTINTAKE)
                                .deadlineWith(new FourBarCommand(fourBar, () -> Constants.fourBarOut)),
                        new FourBarCommand(fourBar, () -> Constants.fourBarHome)
                                .alongWith(new InstantCommand(() -> driverRaw.setRumble(RumbleType.kBothRumble, 1)))));
        driver.leftTrigger()
                .onFalse(new IntakeCommand(intake, () -> 0.0, IntakeMode.FORCEINTAKE)
                        .alongWith(new FourBarCommand(fourBar, () -> Constants.fourBarHome))
                        .alongWith(new InstantCommand(() -> driverRaw.setRumble(RumbleType.kBothRumble, 0))));

        // // // shoot command
        driver.leftBumper().whileTrue(new IntakeCommand(intake, () -> 10.0, IntakeMode.FORCEINTAKE));
        driver.leftBumper().onFalse(new IntakeCommand(intake, () -> 0.0, IntakeMode.FORCEINTAKE));

        // // spit command
        driver.rightBumper().whileTrue(new IntakeCommand(intake, () -> -0.4, IntakeMode.FORCEINTAKE));
        driver.rightBumper().onFalse(new IntakeCommand(intake, () -> 0.0, IntakeMode.FORCEINTAKE));

        // // spin shooter command
        copilot.leftTrigger()
                .whileTrue(new ShooterCommand(
                        shooter,
                        () -> AutoAim.calculateShooterRPM(
                                () -> drivetrain.getState().Pose,
                                () -> new Translation2d(
                                        drivetrain.getState().speeds.vxMetersPerSecond,
                                        drivetrain.getState().speeds.vyMetersPerSecond)),
                        () -> AutoAim.calculateShooterRPM(
                                () -> drivetrain.getState().Pose,
                                () -> new Translation2d(
                                        drivetrain.getState().speeds.vxMetersPerSecond,
                                        drivetrain.getState().speeds.vyMetersPerSecond))));
        copilot.leftTrigger().onFalse(new InstantCommand(() -> shooter.stop()));

        // cop9lot shoot
        copilot.rightBumper().whileTrue(new IntakeCommand(intake, () -> 1.0, IntakeMode.FORCEINTAKE));
        copilot.rightBumper().onFalse(new IntakeCommand(intake, () -> 0.0, IntakeMode.FORCEINTAKE));

        // stop shoot
        // copilot.leftBumper()
        //         .whileTrue(new IntakeCommand(
        //                 intake,
        //                 () -> ((Math.sqrt(Math.pow(drivetrain.getState().speeds.vxMetersPerSecond, 2)
        //                                         + Math.pow(drivetrain.getState().speeds.vyMetersPerSecond, 2))
        //                                 < Constants.stillShotSpeed)
        //                         ? 1.0
        //                         : 0.0),
        //                 IntakeMode.FORCEINTAKE));
        // copilot.leftBumper().onFalse(new IntakeCommand(intake, () -> 0.0, IntakeMode.FORCEINTAKE));

        // point blank aim
        copilot.leftBumper()
                .whileTrue(new FourBarCommand(fourBar, () -> Constants.fourBarOut)
                        .alongWith(new ShooterCommand(shooter, () -> 4500.0, () -> 4500.0)));
        copilot.leftBumper()
                .onFalse(new FourBarCommand(fourBar, () -> Constants.fourBarHome)
                        .alongWith(new InstantCommand(() -> shooter.stop())));

        copilot.leftStick().whileTrue(new FourBarCommand(fourBar, () -> 3.0));
        copilot.leftStick().onFalse(new FourBarCommand(fourBar, () -> Constants.fourBarHome));

        // // // aim command
        copilot.rightTrigger()
                .whileTrue(new FourBarCommand(
                                fourBar,
                                () -> AutoAim.calculateFourBarPosition(
                                        () -> drivetrain.getState().Pose,
                                        () -> new Translation2d(
                                                drivetrain.getState().speeds.vxMetersPerSecond,
                                                drivetrain.getState().speeds.vyMetersPerSecond)))
                        .alongWith(drivetrain.applyRequest(
                                () -> drive.withRotationalRate(calculateAutoTurn(() -> AutoAim.calculateAngleToSpeaker(
                                                        () -> drivetrain.getState().Pose,
                                                        () -> new Translation2d(
                                                                drivetrain.getState().speeds.vxMetersPerSecond,
                                                                drivetrain.getState().speeds.vyMetersPerSecond))
                                                .getDegrees()))
                                        .withVelocityX(xLimiter.calculate(
                                                -JoystickMap.JoystickPowerCalculate(driver.getRightY()) * MaxSpeed))
                                        .withVelocityY(yLimiter.calculate(
                                                -JoystickMap.JoystickPowerCalculate(driver.getRightX()) * MaxSpeed)))));
        copilot.rightTrigger().onFalse(new FourBarCommand(fourBar, () -> Constants.fourBarHome));

        // // // vision-assisted intake command
        // if (noteVisionSubsystem.hasTargets()) {

        // driver.rightTrigger()
        //             .whileTrue(
        //                 new IntakeCommand
        //                                 intake,
        //                                 () -> ((driverRaw.getLeftTriggerAxis() - 0.5) * 2),
        //                                 IntakeMode.SOFTINTAKE)
        //                         .deadlineWith(new ParallelCommandGroup(new FourBarCommand(fourBar, () ->
        // Constants.fourBarOut),
        //                         (drivetrain.applyRequest(() -> drive.withVelocityX(xLimiter.calculate(0.1
        //                                     * MaxSpeed
        //                                     * Math.cos(Units.degreesToRadians(noteVisionSubsystem.getYawVal()))))
        //                             .withVelocityY(yLimiter.calculate(-0.1
        //                                     * MaxSpeed
        //                                     * Math.sin(Units.degreesToRadians(noteVisionSubsystem.getYawVal()))))
        //                             .withRotationalRate(
        //                                     calculateAutoTurn(() -> noteVisionSubsystem.getYawVal()))))));

        // driver.rightTrigger()
        //         .onFalse(new IntakeCommand(intake, () -> 0.0, IntakeMode.FORCEINTAKE)
        //                 .alongWith(new FourBarCommand(fourBar, () -> Constants.fourBarHome)));
        // }

        // // vision-assisted following command
        // driver.rightTrigger().whileTrue((
        //     drivetrain.applyRequest(() -> drive
        //         .withVelocityX(xLimiter.calculate(0.1 *MaxSpeed*
        // Math.cos(Units.degreesToRadians(noteVisionSubsystem.getYawVal()))))
        //         .withVelocityY(yLimiter.calculate(-0.1 *MaxSpeed*
        // Math.sin(Units.degreesToRadians(noteVisionSubsystem.getYawVal()))))
        //         .withRotationalRate(zLimiter.calculate(calculateAutoTurn(() -> noteVisionSubsystem.getYawVal()))))));

        // transfer spin up
        copilot.b().whileTrue(new ShooterCommand(shooter, () -> 1650.0, () -> 1650.0));
        copilot.b().onFalse(new InstantCommand(() -> shooter.stop()));

        // // arm commands
        copilot.a()
                .onTrue(new ArmCommand(
                        arm, () -> ArmSetPoints.home, () -> ArmSetPoints.homeWrist + copilot.getLeftX(), () -> false));
        copilot.b()
                .whileTrue(new ArmCommand(arm, () -> ArmSetPoints.pickup, () -> ArmSetPoints.pickupWrist, () -> false));
        copilot.b()
                .onFalse(new ArmCommand(
                        arm, () -> ArmSetPoints.home, () -> ArmSetPoints.homeWrist + copilot.getLeftX(), () -> false));
        copilot.y()
                .whileTrue(new ArmCommand(
                        arm,
                        () -> ArmSetPoints.dropoff.plus(new Translation2d(
                                (DriverStation.getAlliance().get() == Alliance.Red ? -1 : 1)
                                        * copilot.getLeftX()
                                        * ArmSetPoints.dropoffMultiplier,
                                -copilot.getLeftY() * ArmSetPoints.dropoffMultiplierY)),
                        () -> ArmSetPoints.dropoffWrist,
                        () -> false));
        copilot.y()
                .onFalse(new ArmCommand(
                        arm, () -> ArmSetPoints.home, () -> ArmSetPoints.homeWrist + copilot.getLeftX(), () -> false));

        copilot.back().whileTrue(new IntakeCommand(intake, () -> -0.4, IntakeMode.FORCEINTAKE));
        copilot.back().onFalse(new IntakeCommand(intake, () -> 0.0, IntakeMode.FORCEINTAKE));

        copilot.povUp()
                .onTrue(new ArmCommand(
                                arm,
                                () -> ClimbSetPoints.ready,
                                () -> ClimbSetPoints.readyWrist + copilot.getRightX(),
                                () -> false)
                        // .alongWith(new FourBarCommand(fourBar, () -> Constants.fourBarOut))
                        .alongWith(new InstantCommand(() -> arm.setGains(false))));
        copilot.povRight()
                .onTrue(new ArmCommand(
                        arm, () -> ClimbSetPoints.forward, () -> ClimbSetPoints.forwardWrist, () -> false));
        copilot.povDown()
                .onTrue(new ArmCommand(arm, () -> ClimbSetPoints.down, () -> ClimbSetPoints.downWrist, () -> true)
                        .alongWith(new InstantCommand(() -> arm.setGains(true))));

        copilot.povLeft()
                .onTrue(new ArmCommand(arm, () -> ClimbSetPoints.pinch, () -> ClimbSetPoints.pinchWrist, () -> false));

        // trap
        copilot.rightStick()
                .onTrue(new ArmCommand(
                                arm,
                                () -> ClimbSetPoints.trap.plus(
                                        new Translation2d(copilot.getLeftY() * ClimbSetPoints.trapMultiplier, 0)),
                                () -> ClimbSetPoints.trapwrist,
                                () -> true)
                        .alongWith(new InstantCommand(() -> arm.setGains(false))));

        // reset buttons
        copilot.start()
                .whileTrue(new InstantCommand(() -> arm.resetEncoders())
                        .alongWith(new InstantCommand(() -> arm.setGains(false))));

        // copilot.x().onTrue(new ArmResetCommand(arm, false).alongWith(new InstantCommand(() -> arm.setGains(false))));
        copilot.x().whileTrue(new ShooterCommand(shooter, () -> 4700.0, () -> 5000.0).alongWith(new FourBarCommand(fourBar, () -> 8.0))); // speed1 = CAN ID 6 = top motor
        copilot.x().onFalse(new ShooterCommand(shooter, () -> 0.0, () -> 0.0).alongWith(new FourBarCommand(fourBar, () -> Constants.fourBarHome)));
    }
    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void configureDashboard() {
    /**** Driver tab ****/

    // /**** Vision tab ****/
    final var visionTab = Shuffleboard.getTab("Vision");

    // Pose estimation
    drivetrain.addDashboardWidgets(visionTab);
    }

    public double calculateAutoTurn(Supplier<Double> target) {
        double currentAngle = -(gyro.getAngle() - gyroOffset);

        if (target.get() != 0) {
            targetAngle = -target.get();
        } else if (driverRaw.getPOV() != -1) {
            targetAngle = -driverRaw.getPOV();
        } else if (Math.abs(driver.getLeftX()) >= 0.1 || Math.abs(driver.getLeftY()) >= 0.1) {
            targetAngle = currentAngle - 10 * driver.getLeftX();
            return -driver.getLeftX() * Math.abs(driver.getLeftX()) * 5;
            // targetAngle = (180.0 / Math.PI) * (Math.atan2(-driver.getLeftX(), -driver.getLeftY()));
        }

        double error = (targetAngle - currentAngle) % (360.0);

        error = error > 180.0 ? error - 360.0 : error;
        error = error < -180.0 ? error + 360.0 : error;
        targetAngle = currentAngle + error;
        SmartDashboard.putNumber("angle error deg", error);
        return Math.min(
                Math.max(
                        zLimiter.calculate(gyroPid.calculate(currentAngle, targetAngle)) * MaxAngularRate,
                        -Constants.autoTurnCeiling),
                Constants.autoTurnCeiling);
    }

    public void resetGyro() {
        gyroPid.setIZone(Constants.IZone);
        gyroOffset = gyro.getAngle();
        targetAngle = 0;
        drivetrain.seedFieldRelative(
                (DriverStation.getAlliance().get() == Alliance.Red)
                        ? (new Pose2d(13, 5, new Rotation2d(Math.PI)))
                        : new Pose2d());
    }
}
