// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.ArmCommand;
import frc.robot.Commands.ArmResetCommand;
import frc.robot.Commands.FourBarCommand;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.ShooterCommand;
import frc.robot.Constants.ArmConstants.ArmSetPoints;
import frc.robot.Generated.TunerConstants;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Arm.ArmIO;
import frc.robot.Subsystems.Arm.ArmIOSparkMax;
import frc.robot.Subsystems.FourBar.FourBar;
import frc.robot.Subsystems.FourBar.FourBarIO;
import frc.robot.Subsystems.FourBar.FourBarIOSparkMax;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.Intake.IntakeIOSparkMax;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterIO;
import frc.robot.Subsystems.Shooter.ShooterIOSparkMax;

public class RobotContainer {
    private double MaxSpeed = 6; // 6 meters per second desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
    // driving in open loop
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final SendableChooser<Command> autoChooser;

    private final Intake intake;
    private final Shooter shooter;
    private final FourBar fourBar;
    private final Arm arm;

    private void configureBindings() {
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () -> drive.withVelocityX(
                                        -frc.robot.Util.JoystickMap.JoystickPowerCalculate(joystick.getLeftY())
                                                * MaxSpeed) // Drive forward with
                                // negative Y (forward)
                                .withVelocityY(-frc.robot.Util.JoystickMap.JoystickPowerCalculate(joystick.getLeftX())
                                        * MaxSpeed) // Drive left with negative X (left)
                                .withRotationalRate(-joystick.getRightX()
                                        * MaxAngularRate) // Drive counterclockwise with negative X (left)
                        ));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b()
                .whileTrue(drivetrain.applyRequest(
                        () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                intake = new Intake(new IntakeIOSparkMax()); // Spark Max
                shooter = new Shooter(new ShooterIOSparkMax());
                fourBar = new FourBar(new FourBarIOSparkMax());
                arm = new Arm(new ArmIOSparkMax());
                break;

            default:
                // Replayed robot, disable IO implementations
                intake = new Intake(new IntakeIO() {});
                shooter = new Shooter(new ShooterIO() {});
                fourBar = new FourBar(new FourBarIO() {});
                arm = new Arm(new ArmIO() {});

                break;
        }
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        Shuffleboard.getTab("Autonomous").add(autoChooser);

        intake.setDefaultCommand(new IntakeCommand(intake, () -> joystick.getLeftTriggerAxis()));
        shooter.setDefaultCommand(new ShooterCommand(
                shooter, () -> joystick.getRightTriggerAxis(), () -> joystick.getRightTriggerAxis()));
        fourBar.setDefaultCommand(new FourBarCommand(fourBar, () -> joystick.getLeftX()));
        joystick.a().onTrue(new ArmCommand(arm, () -> ArmSetPoints.home));
        joystick.b().onTrue(new ArmCommand(arm, () -> ArmSetPoints.pickup));
        joystick.x().onTrue(new ArmCommand(arm, () -> ArmSetPoints.amp));
        joystick.y().onTrue(new ArmResetCommand(arm));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
