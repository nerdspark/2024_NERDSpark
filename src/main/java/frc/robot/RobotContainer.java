// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.FourBarCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.IntakeCommand.IntakeMode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
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
    private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
    private final XboxController joystickNonCommand = new XboxController(0);
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

    private void configureBindings() {
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () -> drive.withVelocityX(xLimiter.calculate(
                                        -frc.robot.util.JoystickMap.JoystickPowerCalculate(joystick.getRightY())
                                                * MaxSpeed)) // Drive forward with
                                // negative Y (forward)
                                .withVelocityY(yLimiter.calculate(
                                        -frc.robot.util.JoystickMap.JoystickPowerCalculate(joystick.getRightX())
                                                * MaxSpeed)) // Drive left with negative X (left)
                                .withRotationalRate(zLimiter.calculate(calculateAutoTurn(() -> 0.0)
                                        * MaxAngularRate)) // Drive counterclockwise with negative X (left)
                        ));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b()
                .whileTrue(drivetrain.applyRequest(
                        () -> point.withModuleDirection(new Rotation2d(-joystick.getRightY(), -joystick.getRightX()))));

        // reset the field-centric heading on left bumper press
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        drivetrain.registerTelemetry(logger::telemeterize);
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

        // joystick.a().whileTrue(new ParallelCommandGroup(new IntakeCommand(intake, () -> 0.5, IntakeMode.SOFTINTAKE).andThen(new FourBarCommand(fourBar, () -> Constants.fourBarHome)), new FourBarCommand(fourBar, () -> Constants.fourBarOut)));
        // joystick.a().onFalse(new FourBarCommand(fourBar, () -> Constants.fourBarHome));
        // joystick.b().whileTrue(new IntakeCommand(intake, () -> 1.0, IntakeMode.FORCEINTAKE));
        // joystick.x().whileTrue(new IntakeCommand(intake, () -> -1.0, IntakeMode.FORCEINTAKE));
        // joystick.y().whileTrue(new ParallelCommandGroup(new ShooterCommand(shooter, () -> 1.0, () -> 1.0),new FourBarCommand(fourBar,() -> 0.0), new InstantCommand(() -> calculateAutoTurn(() -> ShooterMap.getAngle(drivetrain.getDaqThread().getClass())))));
        // joystick.rightBumper().whileTrue(new InstantCommand (() -> drive.withRotationalRate(calculateAutoTurn(() -> aprilTagVision.getCurrentCommand().getSubsystem()))));
        double targ = 70;
        // joystick.a().whileTrue(new IntakeCommand(intake, () -> 0.8, IntakeMode.SOFTINTAKE).deadlineWith(drivetrain.applyRequest(() -> drive.withVelocityX(-0.1 *MaxSpeed* Math.cos(targ)).withVelocityY(0.1 *MaxSpeed* Math.sin(targ)).withRotationalRate(calculateAutoTurn(() -> targ)))));
        joystick.a().whileTrue((drivetrain.applyRequest(() -> drive.withVelocityX(0.1 *MaxSpeed* Math.cos(Units.degreesToRadians(-targ))).withVelocityY(0.1 *MaxSpeed* Math.sin(Units.degreesToRadians(-targ))).withRotationalRate(calculateAutoTurn(() -> targ)))));
        
        // intake.setDefaultCommand(new IntakeCommand(intake, () -> joystick.getLeftTriggerAxis(),
        // IntakeMode.SOFTINTAKE));
        // shooter.setDefaultCommand(new ShooterCommand(
        //         shooter, () -> joystick.getRightTriggerAxis(), () -> joystick.getRightTriggerAxis()));
        // fourBar.setDefaultCommand(new FourBarCommand(fourBar, () -> joystick.getLeftX()));
        // joystick.a().onTrue(new ArmCommand(arm, () -> ArmSetPoints.home));
        // joystick.b().onTrue(new ArmCommand(arm, () -> ArmSetPoints.pickup));
        // joystick.x().onTrue(new ArmCommand(arm, () -> ArmSetPoints.amp));
        // arm.setDefaultCommand(new ArmCommand(arm, () -> new
        // Translation2d(Math.atan2(joystick.getLeftX(),joystick.getLeftX()),
        // Math.atan2(joystick.getRightX(),joystick.getRightY()))));
        // joystick.y().onTrue(new ArmResetCommand(arm));
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
            if (joystickNonCommand.getPOV() != -1) {
                targetAngle = -joystickNonCommand.getPOV();
            } else if (Math.abs(joystick.getLeftX()) >= 0.1 || Math.abs(joystick.getLeftY()) >= 0.1) {
                targetAngle = (180.0 / Math.PI) * (Math.atan2(-joystick.getLeftX(), -joystick.getLeftY()));
            }
        }

        double error = (targetAngle - currentAngle) % (360.0);

        error = error > 180.0 ? error - 360.0 : error;
        error = error < -180.0 ? error + 360.0 : error;
        targetAngle = currentAngle + error;
        return gyroPid.calculate(currentAngle, targetAngle);
    }
}
