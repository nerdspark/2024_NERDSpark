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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmSetPoints;
import frc.robot.commands.FourBarCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.IntakeCommand.IntakeMode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.NoteVisionSubsystem;
import frc.robot.subsystems.fourBar.FourBar;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.AprilTagVisionIOPhotonVision;
import frc.robot.util.JoystickMap;
import java.util.function.Supplier;
import java.util.function.Supplier;

public class RobotContainer {
    private double MaxSpeed = 6.0; // 6 meters per second desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    private Intake intake;
    private FourBar fourBar;
    private Shooter shooter;

    private SlewRateLimiter xLimiter = new SlewRateLimiter(3);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(3);
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
    private AprilTagVision aprilTagVision;
    private NoteVisionSubsystem noteVisionSubsystem =
            new NoteVisionSubsystem(Constants.VisionConstants.NOTE_CAMERA_NAME);

    private void configureBindings() {
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () -> drive.withVelocityX(
                                        xLimiter.calculate(-JoystickMap.JoystickPowerCalculate(driver.getRightY())
                                                * MaxSpeed)) // Drive forward with
                                // negative Y (forward)
                                .withVelocityY(
                                        yLimiter.calculate(-JoystickMap.JoystickPowerCalculate(driver.getRightX())
                                                * MaxSpeed)) // Drive left with negative X (left)
                                .withRotationalRate(zLimiter.calculate(calculateAutoTurn(() -> 0.0)
                                        * MaxAngularRate)) // Drive counterclockwise with negative X (left)
                        ));

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b()
                .whileTrue(drivetrain.applyRequest(
                        () -> point.withModuleDirection(new Rotation2d(-driver.getRightY(), -driver.getRightX()))));
        driver.x().onTrue(new InstantCommand(() -> resetGyro()));
        // reset the field-centric heading on left bumper press
        driver.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        drivetrain.registerTelemetry(logger::telemeterize);
        // fourBar.setDefaultCommand(new FourBarCommand(fourBar, () -> Constants.fourBarHome));
    }

    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // intake = new Intake(new IntakeIOSparkMax()); // Spark Max
                // fourBar = new FourBar(new FourBarIOSparkMax());
                // shooter = new Shooter(new ShooterIOSparkMax());
                // arm = new Arm(new ArmIOSparkMax());
                // arm.getArmPosition();
                break;

            default:
                // Replayed robot, disable IO implementations
                // intake = new Intake(new IntakeIO() {});
                // shooter = new Shooter(new ShooterIO() {});
                // fourBar = new FourBar(new FourBarIO() {});
                // arm = new Arm(new ArmIO() {});

                break;
        }
        configureBindings();
        // NamedCommands.registerCommand("IntakeCommand", intake.Intake());

        configureDashboard();
        autoChooser = AutoBuilder.buildAutoChooser();
        Shuffleboard.getTab("Autonomous").add(autoChooser);
        if (Constants.VisionConstants.USE_VISION == true) {
            aprilTagVision = new AprilTagVision(new AprilTagVisionIOPhotonVision());
            aprilTagVision.setDataInterfaces(drivetrain::addVisionData);
        }

        // intake button
        // driver.leftTrigger().whileTrue(
        //     new SequentialCommandGroup(
        //         new IntakeCommand(
        //             intake, () -> driverRaw.getLeftTriggerAxis(),
        // IntakeMode.SOFTINTAKE)
        //                 .deadlineWith(new FourBarCommand(fourBar, () -> Constants.fourBarOut)),
        //             new
        // FourBarCommand(fourBar, () -> Constants.fourBarHome)));

        // fourbar retract, unnecessary
        // joystick.a().onFalse(new FourBarCommand(fourBar, () -> Constants.fourBarHome));

        // shoot command
        // driver.leftBumper().whileTrue(new IntakeCommand(intake, () -> 1.0, IntakeMode.FORCEINTAKE));

        // spit command
        // driver.x().whileTrue(new IntakeCommand(intake, () -> -1.0, IntakeMode.FORCEINTAKE));

        // spin shooter command
        // copilot.y().toggleOnFalse(new ShooterCommand(shooter, () -> 1.0, () -> 1.0));

        // aim command
        // driver.rightBumper().whileTrue(new ParallelCommandGroup(
        //     drive.withRotationalRate(calculateAutoTurn(() -> aimingMap.angle(drivetrain.getState().Pose)))
        //         .withVelocityX(xLimiter.calculate(-JoystickMap.JoystickPowerCalculate(driver.getRightY()) *
        // MaxSpeed))
        //         .withVelocityY(yLimiter.calculate(-JoystickMap.JoystickPowerCalculate(driver.getRightX()) *
        // MaxSpeed)),
        //     new FourBarCommand(fourBar, () -> aimingMap.fourBar(drivetrain.getState().Pose))));

        // vision-assisted intake command
        // if (noteVisionSubsystem.hasTargets()) {
        //     driver.rightTrigger().whileTrue(
        //         new IntakeCommand(intake, () -> driverRaw.getRightTriggerAxis(), IntakeMode.SOFTINTAKE)
        //         .deadlineWith(drivetrain.applyRequest(() -> drive
        //             .withVelocityX(xLimiter.calculate(0.1 *MaxSpeed*
        // Math.cos(Units.degreesToRadians(noteVisionSubsystem.getYawVal()))))
        //             .withVelocityY(yLimiter.calculate(-0.1 *MaxSpeed*
        // Math.sin(Units.degreesToRadians(noteVisionSubsystem.getYawVal()))))
        //             .withRotationalRate(zLimiter.calculate(calculateAutoTurn(() ->
        // noteVisionSubsystem.getYawVal()))))));
        // }
        // vision-assisted following command
        // driver.rightTrigger().whileTrue((
        //     drivetrain.applyRequest(() -> drive
        //         .withVelocityX(xLimiter.calculate(0.1 *MaxSpeed*
        // Math.cos(Units.degreesToRadians(noteVisionSubsystem.getYawVal()))))
        //         .withVelocityY(yLimiter.calculate(-0.1 *MaxSpeed*
        // Math.sin(Units.degreesToRadians(noteVisionSubsystem.getYawVal()))))
        //         .withRotationalRate(zLimiter.calculate(calculateAutoTurn(() -> noteVisionSubsystem.getYawVal()))))));

        // arm commands
        // joystick.a().onTrue(new ArmCommand(arm, () -> ArmSetPoints.home, () -> ArmSetPoints.homeWrist, false));
        // joystick.b().onTrue(new ArmCommand(arm, () -> ArmSetPoints.pickup, () -> ArmSetPoints.pickupWrist, false));
        // joystick.x().onTrue(new ArmCommand(arm, () -> ArmSetPoints.amp, () -> ArmSetPoints.ampWrist, false));
        // joystick.y().onTrue(new ArmCommand(arm, () -> ArmSetPoints.dropoff.plus(new Translation2d(
        //     driver.getLeftY() * ArmSetPoints.dropoffMultiplier, 0)), () -> ArmSetPoints.dropoffWrist, false));
        // arm.setDefaultCommand(new ArmCommand(arm, () -> new
        // Translation2d(Math.atan2(joystick.getLeftX(),joystick.getLeftX()),
        // Math.atan2(joystick.getRightX(),joystick.getRightY()))));
        // joystick.start().onTrue(new ArmResetCommand(arm));
        
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
        double currentAngle = -gyro.getAngle();

        if (target.get() != 0) {
            targetAngle = -target.get();
        } else {
            if (driverRaw.getPOV() != -1) {
                targetAngle = -driverRaw.getPOV();
            } else if (Math.abs(driver.getLeftX()) >= 0.1 || Math.abs(driver.getLeftY()) >= 0.1) {
                targetAngle = (180.0 / Math.PI) * (Math.atan2(-driver.getLeftX(), -driver.getLeftY()));
            }
        }


        double error = (targetAngle - currentAngle) % (360.0);

        error = error > 180.0 ? error - 360.0 : error;
        error = error < -180.0 ? error + 360.0 : error;
        targetAngle = currentAngle + error;
        return gyroPid.calculate(currentAngle, targetAngle);
    }
    public void resetGyro() {
        gyro.reset();
        targetAngle = 0.0;
    }
}
