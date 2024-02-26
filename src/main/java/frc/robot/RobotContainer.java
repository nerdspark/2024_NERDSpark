// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants.ArmSetPoints;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmResetCommand;
import frc.robot.commands.FourBarCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeCommand.IntakeMode;
import frc.robot.commands.ShooterCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.NoteVisionSubsystem;
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
import frc.robot.util.AutoAim;
import frc.robot.util.JoystickMap;
import java.util.function.Supplier;

public class RobotContainer {
    private double MaxSpeed = 6.0; // 6 meters per second desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    private Intake intake;
    private FourBar fourBar;
    private Shooter shooter;
    private Arm arm;

    private SlewRateLimiter xLimiter = new SlewRateLimiter(7);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(7);
    private SlewRateLimiter zLimiter = new SlewRateLimiter(10);
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
    private double gyroOffset = 0;
    private AprilTagVision aprilTagVision;
    private NoteVisionSubsystem noteVisionSubsystem =
            new NoteVisionSubsystem(Constants.VisionConstants.NOTE_CAMERA_NAME);

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
                                .withRotationalRate(zLimiter.calculate(
                                        calculateAutoTurn(() -> 0.0).get()
                                                * MaxAngularRate)) // Drive counterclockwise with negative X (left)
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
        fourBar.setDefaultCommand(new FourBarCommand(fourBar, () -> Constants.fourBarHome));
    }

    public RobotContainer() {

        drivetrain.setRobotIntake(intake);

        // drivetrain.getModule(0).getDriveMotor().setInverted(false);

        // NamedCommands.registerCommand(
        //         "shootSpeed",
        //         new ShooterCommand(
        //                 shooter,
        //                 () -> AutoAim.calculateShooterRPM(() -> drivetrain.getState().Pose),
        //                 () -> AutoAim.calculateShooterRPM(() -> drivetrain.getState().Pose)));

        // NamedCommands.registerCommand("fourBarToIntake", new FourBarCommand(fourBar, () -> Constants.fourBarOut));
        // NamedCommands.registerCommand(
        //         "fourBarToShooter",
        //         new FourBarCommand(fourBar, () -> AutoAim.calculateFourBarPosition(() ->
        // drivetrain.getState().Pose)));
        // NamedCommands.registerCommand(
        //         "forcedIntake", new IntakeCommand(intake, () -> 1.0, IntakeCommand.IntakeMode.FORCEINTAKE));

        // NamedCommands.registerCommand("backToSafety", new backToSafety(intake, fourBar));
        // NamedCommands.registerCommand("intakingRings", new activeIntaking(intake, fourBar));

        // NamedCommands.registerCommand("shootNote1", new firstRing(shooter, fourBar, intake));
        // NamedCommands.registerCommand("shootNote2", new secondRing(fourBar, intake));
        // NamedCommands.registerCommand("shootNote3", new thirdRing(fourBar, intake));

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
        configureBindings();

        configureDashboard();
        autoChooser = AutoBuilder.buildAutoChooser();
        Shuffleboard.getTab("Autonomous").add(autoChooser);
        if (Constants.VisionConstants.USE_VISION == true) {
            aprilTagVision = new AprilTagVision(new AprilTagVisionIOPhotonVision());
            aprilTagVision.setDataInterfaces(drivetrain::addVisionData);
        }

        // intake button
        driver.leftTrigger()
                .whileTrue(new SequentialCommandGroup(
                        new IntakeCommand(
                                        intake,
                                        () -> ((driverRaw.getLeftTriggerAxis() - 0.5) * 2),
                                        IntakeMode.SOFTINTAKE)
                                .deadlineWith(new FourBarCommand(fourBar, () -> Constants.fourBarOut)),
                        new FourBarCommand(fourBar, () -> Constants.fourBarHome)));
        driver.leftTrigger().onFalse(new IntakeCommand(intake, () -> 0.0, IntakeMode.FORCEINTAKE));

        // // // shoot command
        driver.leftBumper().whileTrue(new IntakeCommand(intake, () -> 1.0, IntakeMode.FORCEINTAKE));
        driver.leftBumper().onFalse(new IntakeCommand(intake, () -> 0.0, IntakeMode.FORCEINTAKE));

        // // spit command
        driver.rightBumper().whileTrue(new IntakeCommand(intake, () -> -1.0, IntakeMode.FORCEINTAKE));
        driver.rightBumper().onFalse(new IntakeCommand(intake, () -> 0.0, IntakeMode.FORCEINTAKE));

        // // spin shooter command
        copilot.leftBumper()
                .whileTrue(new ShooterCommand(
                        shooter,
                        () -> AutoAim.calculateShooterRPM(() -> drivetrain.getState().Pose),
                        () -> AutoAim.calculateShooterRPM(() -> drivetrain.getState().Pose)));
        copilot.leftBumper().onFalse(new InstantCommand(() -> shooter.stop()));

        // transfer spin up
        // copilot.leftTrigger()
        //         .whileTrue(new ShooterCommand(
        //                 shooter, () -> 1300 * copilot.getLeftTriggerAxis(), () -> 1300 *
        // copilot.getLeftTriggerAxis()));
        // copilot.leftTrigger().onFalse(new InstantCommand(() -> shooter.stop()));

        // transfer shoot
        copilot.rightTrigger()
                .whileTrue(new IntakeCommand(intake, () -> copilot.getRightTriggerAxis(), IntakeMode.FORCEINTAKE));
        copilot.rightTrigger().onFalse(new IntakeCommand(intake, () -> 0.0, IntakeMode.FORCEINTAKE));
        // // // aim command
        copilot.rightBumper()
                .whileTrue(new ParallelCommandGroup(
                        new InstantCommand(() -> drive.withRotationalRate(calculateAutoTurn(
                                                () -> AutoAim.calculateAngleToSpeaker(() -> 
                                                        drivetrain.getState().Pose//, () -> drivetrain.getState().speeds.vxMetersPerSecond, () -> drivetrain.getState().speeds.vyMetersPerSecond
                                                        )
                                                        .get()
                                                        .getDegrees())
                                        .get())
                                .withVelocityX(xLimiter.calculate(
                                        -JoystickMap.JoystickPowerCalculate(driver.getRightY()) * MaxSpeed))
                                .withVelocityY(yLimiter.calculate(
                                        -JoystickMap.JoystickPowerCalculate(driver.getRightX()) * MaxSpeed))),
                        new FourBarCommand(
                                fourBar, () -> AutoAim.calculateFourBarPosition(() -> drivetrain.getState().Pose))));

        // // // vision-assisted intake command
        // if (noteVisionSubsystem.hasTargets()) {
        //     driver.rightTrigger()
        //             .whileTrue(new IntakeCommand(intake, () -> driverRaw.getRightTriggerAxis(),
        // IntakeMode.SOFTINTAKE)
        //                     .deadlineWith(drivetrain.applyRequest(() -> drive.withVelocityX(xLimiter.calculate(0.1
        //                                     * MaxSpeed
        //                                     * Math.cos(Units.degreesToRadians(noteVisionSubsystem.getYawVal()))))
        //                             .withVelocityY(yLimiter.calculate(-0.1
        //                                     * MaxSpeed
        //                                     * Math.sin(Units.degreesToRadians(noteVisionSubsystem.getYawVal()))))
        //                             .withRotationalRate(zLimiter.calculate(
        //                                     calculateAutoTurn(() -> noteVisionSubsystem.getYawVal()))))));
        // }
        // // vision-assisted following command
        // driver.rightTrigger().whileTrue((
        //     drivetrain.applyRequest(() -> drive
        //         .withVelocityX(xLimiter.calculate(0.1 *MaxSpeed*
        // Math.cos(Units.degreesToRadians(noteVisionSubsystem.getYawVal()))))
        //         .withVelocityY(yLimiter.calculate(-0.1 *MaxSpeed*
        // Math.sin(Units.degreesToRadians(noteVisionSubsystem.getYawVal()))))
        //         .withRotationalRate(zLimiter.calculate(calculateAutoTurn(() -> noteVisionSubsystem.getYawVal()))))));

        // // arm commands
        copilot.a()
                .onTrue(new ArmCommand(
                        arm, () -> ArmSetPoints.home, () -> ArmSetPoints.homeWrist + copilot.getRightX(), () -> false));
        copilot.b().onTrue(new ArmCommand(arm, () -> ArmSetPoints.pickup, () -> ArmSetPoints.pickupWrist, () -> false));
        copilot.x().onTrue(new ArmCommand(arm, () -> ArmSetPoints.amp, () -> ArmSetPoints.ampWrist, () -> false));
        copilot.y()
                .whileTrue(new ArmCommand(
                        arm,
                        () -> ArmSetPoints.dropoff.plus(
                                new Translation2d(copilot.getLeftY() * ArmSetPoints.dropoffMultiplier, 0)),
                        () -> ArmSetPoints.dropoffWrist,
                        () -> false));
        copilot.y()
                .onFalse(new ArmCommand(
                        arm, () -> ArmSetPoints.home, () -> ArmSetPoints.homeWrist + copilot.getRightX(), () -> false));

        copilot.back().whileTrue(new IntakeCommand(intake, () -> -1.0, IntakeMode.FORCEINTAKE));
        copilot.back().onFalse(new IntakeCommand(intake, () -> 0.0, IntakeMode.FORCEINTAKE));

        // arm.setDefaultCommand(new ArmCommand(arm, () -> new
        // Translation2d(Math.atan2(joystick.getLeftX(),joystick.getLeftX()),
        // Math.atan2(joystick.getRightX(),joystick.getRightY()))));
        copilot.start().onTrue(new ArmResetCommand(arm));
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

    public Supplier<Double> calculateAutoTurn(Supplier<Double> target) {
        double currentAngle = -(gyro.getAngle() - gyroOffset);

        if (target.get() != 0) {
            targetAngle = -target.get();
        } else {
            if (driverRaw.getPOV() != -1) {
                targetAngle = -driverRaw.getPOV();
            } else if (Math.abs(driver.getLeftX()) >= 0.1 || Math.abs(driver.getLeftY()) >= 0.1) {
                targetAngle = currentAngle;
                return () -> driver.getLeftX() / 2;
                // targetAngle = (180.0 / Math.PI) * (Math.atan2(-driver.getLeftX(), -driver.getLeftY()));
            }
        }

        double error = (targetAngle - currentAngle) % (360.0);

        error = error > 180.0 ? error - 360.0 : error;
        error = error < -180.0 ? error + 360.0 : error;
        targetAngle = currentAngle + error;
        SmartDashboard.putNumber("angle error deg", error);
        return () -> gyroPid.calculate(currentAngle, targetAngle);
    }

    public void resetGyro() {
        gyroPid.setIZone(Constants.IZone);
        gyroOffset = gyro.getAngle();
        targetAngle = 0;
        drivetrain.seedFieldRelative();
    }
}
