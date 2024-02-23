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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.NoteVisionSubsystem;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.AprilTagVisionIOPhotonVision;

public class RobotContainer {
    private double MaxSpeed = 6; // 6 meters per second desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    private SlewRateLimiter xLimiter = new SlewRateLimiter(3);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(3);
    private SlewRateLimiter zLimiter = new SlewRateLimiter(15);
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

    private NoteVisionSubsystem noteVisionSubsystem = new NoteVisionSubsystem(Constants.VisionConstants.NOTE_CAMERA_NAME);

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
                                .withRotationalRate(zLimiter.calculate(calculateAutoTurn()
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

        if(noteVisionSubsystem.hasTargets()){
            double noteYaw = noteVisionSubsystem.getYawVal();
         }
    }

    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // intake = new Intake(new IntakeIOSparkMax()); // Spark Max
                // shooter = new Shooter(new ShooterIOSparkMax());
                // fourBar = new FourBar(new FourBarIOSparkMax());
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

    public double calculateAutoTurn() {
    double currentAngle = -gyro.getAngle();

        if (joystickNonCommand.getPOV() != -1) {

            targetAngle = joystickNonCommand.getPOV();
        } else if (Math.abs(joystick.getLeftX()) >= 0.1 || Math.abs(joystick.getLeftY()) >= 0.1) {
            targetAngle = (180.0 / Math.PI) * (Math.atan2(joystick.getLeftX(), joystick.getLeftY()));
        }

        double error = (targetAngle - currentAngle) % (360.0);

        error = error > 180.0 ? error - 360.0 : error;
        error = error < -180.0 ? error + 360.0 : error;
        targetAngle = currentAngle + error;
        return gyroPid.calculate(currentAngle, targetAngle);
    }
}
