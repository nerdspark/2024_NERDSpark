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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.FixedShotConstants;
import frc.robot.Constants.FourBarConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.SpeakerConstants;
import frc.robot.actions.activeIntaking;
import frc.robot.actions.backToSafety;
import frc.robot.commands.FourBarCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeCommand.IntakeMode;
import frc.robot.config.RobotIdentity;
import frc.robot.commands.ShooterCommand;
import frc.robot.generated.TunerConstantsSmidge;
import frc.robot.generated.TunerConstantsSmudge;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.arm.Arm;
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
import frc.robot.util.LightningShuffleboard;

import java.util.function.Supplier;

public class RobotContainer { // implements RobotConstants{
    private double MaxSpeed = 6.0; // 6 meters per second desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
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
    private final CommandSwerveDrivetrain drivetrain; 

    private final XboxController driverRaw = new XboxController(0);
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
    // driving in open loop
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    //     private final Telemetry logger = new Telemetry(MaxSpeed);
    private final SendableChooser<Command> autoChooser;
    private PIDController gyroPid =
            new PIDController(DrivetrainConstants.gyroP, DrivetrainConstants.gyroI, DrivetrainConstants.gyroD);
    private double targetAngle = 0;
    private final Pigeon2 gyro = new Pigeon2(RobotMap.pigeonID, "canivore1");
    private double gyroOffset = gyro.getAngle();
    private AprilTagVision aprilTagVision;
    private PoseEstimatorSubsystem poseEstimatorSubSystem;
    //     private NoteVisionSubsystem noteVisionSubsystem =
    //             new NoteVisionSubsystem(Constants.VisionConstants.NOTE_CAMERA_NAME);

    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                intake = new Intake(new IntakeIOSparkMax()); // Spark Max
                fourBar = new FourBar(new FourBarIOSparkMax());
                shooter = new Shooter(new ShooterIOSparkMax());
                break;

            default:
                // Replayed robot, disable IO implementations
                intake = new Intake(new IntakeIO() {});
                shooter = new Shooter(new ShooterIO() {});
                fourBar = new FourBar(new FourBarIO() {});
                // arm = new Arm(new ArmIO() {});
                // break;
        }
        drivetrain =TunerConstantsSmidge.DriveTrain;// RobotIdentity.getIdentity() == RobotIdentity.SMIDGE_2024  ? TunerConstantsSmidge.DriveTrain : TunerConstantsSmudge.DriveTrain;
        
        drivetrain.setRobotIntake(intake);
        drivetrain.getModule(0).getDriveMotor().setInverted(false);
        drivetrain.getModule(1).getDriveMotor().setInverted(true); // FR
        drivetrain.getModule(2).getDriveMotor().setInverted(true); // b
        drivetrain.getModule(3).getDriveMotor().setInverted(true); // b

        configureNamedCommands();

        configureDefaultCommands();

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

        configureButtonBindings();

        //only schedule arm commands if using smudge
        if(RobotIdentity.getIdentity() != RobotIdentity.SMIDGE_2024) {
                arm = new Arm(new ArmIOSparkMax());
                scheduleArmCommands();
        }
        // LightningShuffleboard.setDoubleSupplier("four bar", "distance from speaker", () -> drivetrain
        //         .getState()
        //         .Pose
        //         .getTranslation()
        //         .getDistance(
        //                 DriverStation.getAlliance().get() == Alliance.Blue
        //                         ? SpeakerConstants.speakerLocBlue.getTranslation()
        //                         : SpeakerConstants.speakerLocRed.getTranslation()));
    }

    private void configureDefaultCommands() {
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

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        // drivetrain.registerTelemetry(logger::telemeterize);
        // arm.setDefaultCommand(new ArmCommand(arm, () -> ArmSetPoints.home, () -> ArmSetPoints.homeWrist, () ->
        // false));
        fourBar.setDefaultCommand(new FourBarCommand(fourBar, () -> FourBarConstants.fourBarHome));
    }

    private void configureNamedCommands() {
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

        NamedCommands.registerCommand(
                "fourBarToIntake", new FourBarCommand(fourBar, () -> FourBarConstants.fourBarOut));
        NamedCommands.registerCommand("fourBarToHome", new FourBarCommand(fourBar, () -> FourBarConstants.fourBarHome));

        NamedCommands.registerCommand(
                "fourBarToShooter",
                new FourBarCommand(
                        fourBar,
                        () -> AutoAim.calculateFourBarPosition(
                                () -> drivetrain.getState().Pose,
                                () -> new Translation2d(
                                        drivetrain.getState().speeds.vxMetersPerSecond,
                                        drivetrain.getState().speeds.vyMetersPerSecond))));

        NamedCommands.registerCommand("fourBarToRCR2", new FourBarCommand(fourBar, () -> AutoConstants.redCenterRing2));
        NamedCommands.registerCommand("fourBarToRCR3", new FourBarCommand(fourBar, () -> AutoConstants.redCenterRing3));
        NamedCommands.registerCommand("fourBarToRCR4", new FourBarCommand(fourBar, () -> AutoConstants.redCenterRing4));

        NamedCommands.registerCommand(
                "fourBarToBCR2", new FourBarCommand(fourBar, () -> AutoConstants.blueCenterRing2));
        NamedCommands.registerCommand(
                "fourBarToBCR3", new FourBarCommand(fourBar, () -> AutoConstants.blueCenterRing3));
        NamedCommands.registerCommand(
                "fourBarToBCR4", new FourBarCommand(fourBar, () -> AutoConstants.blueCenterRing4));

        NamedCommands.registerCommand("fourBarToWSR1", new FourBarCommand(fourBar, () -> AutoConstants.weirdSideRing1));
        NamedCommands.registerCommand("fourBarToWSR2", new FourBarCommand(fourBar, () -> AutoConstants.weirdSideRing2));
        NamedCommands.registerCommand("fourBarToWSR3", new FourBarCommand(fourBar, () -> AutoConstants.weirdSideRing3));
        NamedCommands.registerCommand("fourBarToWSR4", new FourBarCommand(fourBar, () -> AutoConstants.weirdSideRing4));

        // NamedCommands.registerCommand("fourBarToRRPShoot", new FourBarCommand(fourBar, () ->
        // AutoConstants.weirdSideRing4));

        NamedCommands.registerCommand(
                "RfourBarToWSR1", new FourBarCommand(fourBar, () -> AutoConstants.red_weirdSideRing1));
        NamedCommands.registerCommand(
                "RfourBarToWSR2", new FourBarCommand(fourBar, () -> AutoConstants.red_weirdSideRing2));
        NamedCommands.registerCommand(
                "RfourBarToWSR3", new FourBarCommand(fourBar, () -> AutoConstants.red_weirdSideRing3));
        NamedCommands.registerCommand(
                "RfourBarToWSR4", new FourBarCommand(fourBar, () -> AutoConstants.red_weirdSideRing4));

        // Choreo Shoot Commands
        NamedCommands.registerCommand(
                "fourBarToBCR5Choreo", new FourBarCommand(fourBar, () -> AutoConstants.blueCenterRing5Choreo));
        NamedCommands.registerCommand(
                "fourBarToBCR6Choreo", new FourBarCommand(fourBar, () -> AutoConstants.blueCenterRing6Choreo));
        NamedCommands.registerCommand(
                "fourBarToBCR62Choreo", new FourBarCommand(fourBar, () -> AutoConstants.blueCenterRing62Choreo));
        NamedCommands.registerCommand(
                "fourBarToBCR7Choreo", new FourBarCommand(fourBar, () -> AutoConstants.blueCenterRing7Choreo));

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
    }

    private void configureButtonBindings() {
        /* DRIVER BINDINGS:
         * left trigger: intake
         * left bumper: shoot
         * right bumper: spit
         * start: zero gyro
         */
        // intake
        driver.leftTrigger()
                .whileTrue(new SequentialCommandGroup(
                        new IntakeCommand(intake, () -> ((1.0)), IntakeMode.SOFTINTAKE)
                                .deadlineWith(new FourBarCommand(fourBar, () -> FourBarConstants.fourBarOut)),
                        new FourBarCommand(fourBar, () -> FourBarConstants.fourBarHome)
                                .alongWith(new InstantCommand(() -> driverRaw.setRumble(RumbleType.kBothRumble, 1)))));
        driver.leftTrigger()
                .onFalse(new IntakeCommand(intake, () -> 0.0, IntakeMode.FORCEINTAKE)
                        .alongWith(new FourBarCommand(fourBar, () -> FourBarConstants.fourBarHome))
                        .alongWith(new InstantCommand(() -> driverRaw.setRumble(RumbleType.kBothRumble, 0))));

        // driver shoot
        driver.rightTrigger().whileTrue(new IntakeCommand(intake, () -> 1.0, IntakeMode.FORCEINTAKE));
        driver.rightTrigger().onFalse(new IntakeCommand(intake, () -> 0.0, IntakeMode.FORCEINTAKE));

        // spit
        driver.rightBumper().whileTrue(new IntakeCommand(intake, () -> -0.4, IntakeMode.FORCEINTAKE));
        driver.rightBumper().onFalse(new IntakeCommand(intake, () -> 0.0, IntakeMode.FORCEINTAKE));

        //autoaim robot
        driver.leftBumper().whileTrue(drivetrain.applyRequest(
                                () -> drive.withRotationalRate(calculateAutoTurn(() -> AutoAim.calculateAngleToSpeaker(
                                                        () -> drivetrain.getState().Pose,
                                                        () -> new Translation2d(
                                                                drivetrain.getState().speeds.vxMetersPerSecond,
                                                                drivetrain.getState().speeds.vyMetersPerSecond))
                                                .getDegrees()))
                                        .withVelocityX(xLimiter.calculate(
                                                -JoystickMap.JoystickPowerCalculate(driver.getRightY()) * MaxSpeed))
                                        .withVelocityY(yLimiter.calculate(
                                                -JoystickMap.JoystickPowerCalculate(driver.getRightX()) * MaxSpeed))));

        // zero gyro
        driver.start().onTrue(new InstantCommand(() -> resetGyro()));
        driver.a().onTrue(new InstantCommand(() -> printSpeakerDistanceAndAngle(aprilTagVision)));

        /* COPILOT COMMANDS:
         * left trigger: full auto aim 4 bar + shooter
         * right trigger: full auto vision turn
         * left bumper: spit
         * right bumper: shoot
         * 
         * back: home position FIXED SHOT
         * start:long position FIXED SHOT 
         *
         * a: point blank FIXED SHOT 
         * x: stage safe zone FIXED SHOT 
         * b: arm amp pickup
         * y: arm amp dropoff
         * POV UP: climb up
         * POV down: climb down
         * POV left: trap
         * 
         */

        // NEW COMMAND BINDINGS

        //autoaim robot
        copilot.rightTrigger().whileTrue(drivetrain.applyRequest(
                                () -> drive.withRotationalRate(calculateAutoTurn(() -> AutoAim.calculateAngleToSpeaker(
                                                        () -> drivetrain.getState().Pose,
                                                        () -> new Translation2d(
                                                                drivetrain.getState().speeds.vxMetersPerSecond,
                                                                drivetrain.getState().speeds.vyMetersPerSecond))
                                                .getDegrees()))
                                        .withVelocityX(xLimiter.calculate(
                                                -JoystickMap.JoystickPowerCalculate(driver.getRightY()) * MaxSpeed))
                                        .withVelocityY(yLimiter.calculate(
                                                -JoystickMap.JoystickPowerCalculate(driver.getRightX()) * MaxSpeed))));
        
        //4 bar + shooter autoaim
        copilot.leftTrigger().whileTrue(new ShooterCommand(
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
                                        drivetrain.getState().speeds.vyMetersPerSecond)))
                .alongWith(new FourBarCommand(
                                fourBar,
                                () -> AutoAim.calculateFourBarPosition(
                                        () -> drivetrain.getState().Pose,
                                        () -> new Translation2d(
                                                drivetrain.getState().speeds.vxMetersPerSecond,
                                                drivetrain.getState().speeds.vyMetersPerSecond)))));
        copilot.leftTrigger().onFalse(new InstantCommand(() -> shooter.stop()));

        // copilot shoot
        copilot.rightBumper().whileTrue(new IntakeCommand(intake, () -> 1.0, IntakeMode.FORCEINTAKE));
        copilot.rightBumper().onFalse(new IntakeCommand(intake, () -> 0.0, IntakeMode.FORCEINTAKE));

        // copilot spit
        copilot.leftBumper().whileTrue(new IntakeCommand(intake, () -> -0.4, IntakeMode.FORCEINTAKE));
        copilot.leftBumper().onFalse(new IntakeCommand(intake, () -> 0.0, IntakeMode.FORCEINTAKE));

        //TEST COMMAND
        copilot.back()
                .whileTrue(new FourBarCommand(fourBar, () -> FixedShotConstants.fourBarLong)
                        .alongWith(new ShooterCommand(shooter, () -> 4500.0, () -> 4800.0)));
        copilot.back()
                .onFalse(new InstantCommand(shooter::stop));

        // 4 bar + shooter FIXED shots

        copilot.a()
                .whileTrue(new FourBarCommand(fourBar, () -> FourBarConstants.fourBarOut)
                        .alongWith(new ShooterCommand(shooter, () -> FixedShotConstants.RPMPointBlank, () -> FixedShotConstants.RPMPointBlank)));
        copilot.a()
                .onFalse(new InstantCommand(shooter::stop));     


        copilot.start()
                .whileTrue(new FourBarCommand(fourBar, () -> FixedShotConstants.fourBarLong)
                        .alongWith(new ShooterCommand(shooter, () -> FixedShotConstants.RPMLong, () -> FixedShotConstants.RPMLong)));
        copilot.start()
                .onFalse(new InstantCommand(shooter::stop));    


        copilot.x()
                .whileTrue(new ShooterCommand(shooter, () -> FixedShotConstants.RPMHome, () -> FixedShotConstants.RPMHome));
        copilot.x()
                .onFalse(new InstantCommand(shooter::stop));  

        // aim command
        // TODO: update these positions to non-magic numbers, and for our new position conversion factor
        // copilot.x().whileTrue(new ShooterCommand(shooter, () -> 4700.0, () -> 5000.0).alongWith(new
        // FourBarCommand(fourBar, () -> 8.0))); // speed1 = CAN ID 6 = top motor
        // copilot.x().onFalse(new ShooterCommand(shooter, () -> 0.0, () -> 0.0).alongWith(new FourBarCommand(fourBar,
        // () -> FourBarConstants.fourBarHome)));

        /* UNUSED */

        // driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driver.b()
        //         .whileTrue(drivetrain.applyRequest(
        //                 () -> point.withModuleDirection(new Rotation2d(-driver.getRightY(), -driver.getRightX()))));
        // reset the field-centric heading on left bumper press
        // driver.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

        // vision-assisted intake command
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
        // copilot.b().whileTrue(new ShooterCommand(shooter, () -> 1650.0, () -> 1650.0));
        // copilot.b().onFalse(new InstantCommand(shooter::stop));

        // stop shoot
        // copilot.leftBumper()
        //         .whileTrue(new IntakeCommand(
        //                 intake,
        //                 () -> ((Math.sqrt(Math.pow(drivetrain.getState().speeds.vxMetersPerSecond, 2)
        //                                         + Math.pow(drivetrain.getState().speeds.vyMetersPerSecond, 2))
        //                                 < ShooterConstants.stillShotSpeed)
        //                         ? 1.0
        //                         : 0.0),
        //                 IntakeMode.FORCEINTAKE));
        // copilot.leftBumper().onFalse(new IntakeCommand(intake, () -> 0.0, IntakeMode.FORCEINTAKE));

    }

    private void scheduleArmCommands() {
        // arm commands
        // copilot.a().onTrue(new ArmCommand(arm, () -> ArmSetPoints.home, () -> ArmSetPoints.homeWrist +
        // copilot.getLeftX(), () -> false));
        // copilot.b().whileTrue(new ArmCommand(arm, () -> ArmSetPoints.pickup, () -> ArmSetPoints.pickupWrist, () ->
        // false));
        // copilot.b().onFalse(new ArmCommand(
        //         arm, () -> ArmSetPoints.home, () -> ArmSetPoints.homeWrist + copilot.getLeftX(), () -> false));
        //         copilot.y().whileTrue(new ArmCommand(
        //                 arm,
        //                 () -> ArmSetPoints.dropoff.plus(new Translation2d(
        //                         (DriverStation.getAlliance().get() == Alliance.Red ? -1 : 1)
        //                                 * copilot.getLeftX()
        //                                 * ArmSetPoints.dropoffMultiplier,
        //                                 -copilot.getLeftY() * ArmSetPoints.dropoffMultiplierY)),
        //                 () -> ArmSetPoints.dropoffWrist,
        //                 () -> false));
        // copilot.y().onFalse(new ArmCommand(arm, () -> ArmSetPoints.home, () -> ArmSetPoints.homeWrist +
        // copilot.getLeftX(), () -> false));

        // copilot.povUp().onTrue(new ArmCommand(
        //                         arm,
        //                         () -> ClimbSetPoints.ready,
        //                         () -> ClimbSetPoints.readyWrist + copilot.getRightX(),
        //                         () -> false)
        //                 // .alongWith(new FourBarCommand(fourBar, () -> Constants.fourBarOut))
        //                 .alongWith(new InstantCommand(() -> arm.setGains(false))));
        // copilot.povRight().onTrue(new ArmCommand(arm, () -> ClimbSetPoints.forward, () ->
        // ClimbSetPoints.forwardWrist, () -> false));
        // copilot.povDown().onTrue(new ArmCommand(arm, () -> ClimbSetPoints.down, () -> ClimbSetPoints.downWrist, () ->
        // true)
        //                 .alongWith(new InstantCommand(() -> arm.setGains(true))));

        // copilot.povLeft().onTrue(new ArmCommand(arm, () -> ClimbSetPoints.pinch, () -> ClimbSetPoints.pinchWrist, ()
        // -> false));

        // // trap
        // copilot.rightStick().onTrue(new ArmCommand(
        //                         arm,
        //                         () -> ClimbSetPoints.trap.plus(
        //                                 new Translation2d(copilot.getLeftY() * ClimbSetPoints.trapMultiplier, 0)),
        //                         () -> ClimbSetPoints.trapwrist,
        //                         () -> true)
        //                 .alongWith(new InstantCommand(() -> arm.setGains(false))));

        // // reset buttons
        // copilot.start().whileTrue(new InstantCommand(() -> arm.resetEncoders()).alongWith(new InstantCommand(() ->
        // arm.setGains(false))));
    }

    //     @Override
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void configureDashboard() {
        /**** Driver tab ****/

        // /**** Vision tab ****/
        final var visionTab = Shuffleboard.getTab("Vision");

        // Pose estimation
        drivetrain.addDashboardWidgets(visionTab);

        // final var driverTab = Shuffleboard.getTab("Driver");
        visionTab.addBoolean("SHOOT", () -> /*shooter.onTarget() &&*/ fourBar.onTarget()); //fourbar and shooter within tolerance of target values
        visionTab.addBoolean("SYNCED", () -> drivetrain.getCurrentPose().getTranslation().getDistance(aprilTagVision.getVisionPose().getTranslation()) < DrivetrainConstants.poseSyncTolerance); //vision pose within tolerance of estimated pose
        visionTab.addBoolean("COLLECTED", intake::getBeamBreak); //collector beambreak triggered

    }

    //     @Override
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
                        -DrivetrainConstants.autoTurnCeiling),
                DrivetrainConstants.autoTurnCeiling);
    }

    //     @Override
    public void resetGyro() {
        gyroPid.setIZone(DrivetrainConstants.IZone);
        gyroOffset = gyro.getAngle();
        targetAngle = 0;
        drivetrain.seedFieldRelative(
                (DriverStation.getAlliance().get() == Alliance.Red)
                        ? (new Pose2d(13, 5, new Rotation2d(Math.PI)))
                        : new Pose2d());
    }

    public void printSpeakerDistanceAndAngle(AprilTagVision aprilTagVision) {

        SmartDashboard.putNumber("Speaker Distance", aprilTagVision.getSpeakerTagDistance());
        SmartDashboard.putNumber("Speaker Angle", aprilTagVision.getSpeakerTagAngle());
    }
}
