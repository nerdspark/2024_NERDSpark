// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.AmpSetpoints;
import frc.robot.Constants.ArmConstants.BlockSetpoints;
import frc.robot.Constants.ArmConstants.PickupSetpoints;
import frc.robot.Constants.ArmConstants.TrapSetpoints;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.FixedShotConstants;
import frc.robot.Constants.FourBarConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.actions.activeIntaking;
import frc.robot.actions.backToSafety;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmCommandAngles;
import frc.robot.commands.BlinkinCommand;
import frc.robot.commands.FourBarCommand;
import frc.robot.commands.GrapplerCommand;
import frc.robot.commands.GripperIndexCommand;
import frc.robot.commands.GripperOutCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeCommand.IntakeMode;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.WinchCommand;
import frc.robot.generated.TunerConstantsSmidge;
import frc.robot.generated.TunerConstantsSmudge;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.blikinLights.BlinkinLights;
import frc.robot.subsystems.blikinLights.BlinkinLightsIOSparkMax;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIOSparkMax;
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
import java.util.function.Supplier;

public class RobotContainer { // implements RobotConstants{
    private double MaxSpeed = 6.0; // 6 meters per second desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // DO NOT CHANGE
    private Intake intake;
    private FourBar fourBar;
    private Shooter shooter;
    private Arm arm;
    private Climb climb;
    private AutoAim m_AutoAim;
    private boolean enableSlowMode = false;
    private BlinkinLights lights;

    private SlewRateLimiter xLimiter = new SlewRateLimiter(8);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(8);
    private SlewRateLimiter zLimiter = new SlewRateLimiter(25);
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandXboxController driver = new CommandXboxController(0); // My joystick
    private final CommandXboxController copilot = new CommandXboxController(1);
    private final CommandSwerveDrivetrain drivetrain;

    private final XboxController driverRaw = new XboxController(0);
    private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
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
    // private NoteVisionSubsystem noteVisionSubsystem =
    //         new NoteVisionSubsystem(Constants.VisionConstants.NOTE_CAMERA_NAME);

    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                intake = new Intake(new IntakeIOSparkMax()); // Spark Max
                fourBar = new FourBar(new FourBarIOSparkMax());
                shooter = new Shooter(new ShooterIOSparkMax());
                m_AutoAim = new AutoAim();
                break;

            default:
                // Replayed robot, disable IO implementations
                intake = new Intake(new IntakeIO() {});
                shooter = new Shooter(new ShooterIO() {});
                fourBar = new FourBar(new FourBarIO() {});
                // arm = new Arm(new ArmIO() {});
                // break;
        }
        if (Constants.PracticeBot) {
            drivetrain = TunerConstantsSmidge.DriveTrain;

            drivetrain.getModule(0).getDriveMotor().setInverted(false); // fl
            drivetrain.getModule(1).getDriveMotor().setInverted(true); // FR
            drivetrain.getModule(2).getDriveMotor().setInverted(true); // bL

            drivetrain.getModule(3).getDriveMotor().setInverted(true); // br
        } else {
            drivetrain = TunerConstantsSmudge.DriveTrain;

            drivetrain.getModule(0).getDriveMotor().setInverted(false); // fl
            drivetrain.getModule(1).getDriveMotor().setInverted(true); // FR
            drivetrain.getModule(2).getDriveMotor().setInverted(false); // bL
            drivetrain.getModule(3).getDriveMotor().setInverted(true); // br
            climb = new Climb(new ClimbIOSparkMax());
            arm = new Arm(new ArmIOSparkMax());
            lights = new BlinkinLights(new BlinkinLightsIOSparkMax());
            scheduleArmCommands();
        }

        drivetrain.setRobotIntake(intake);

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
            drivetrain.setAprilTagVision(aprilTagVision);
        }

        configureButtonBindings();

        // only schedule arm commands if using smudge
        // if(RobotIdentity.getIdentity() != RobotIdentity.SMIDGE_2024) {
        //         arm = new Arm(new ArmIOSparkMax());
        //         scheduleArmCommands();
        // }

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
                drivetrain.applyRequest(() -> drive.withVelocityX(
                                xLimiter.calculate(-JoystickMap.JoystickPowerCalculate(driver.getRightY()) * MaxSpeed))
                        .withVelocityY(
                                yLimiter.calculate(-JoystickMap.JoystickPowerCalculate(driver.getRightX()) * MaxSpeed))
                        .withRotationalRate(calculateAutoTurn(() -> 0.0))));

        // if (Utils.isSimulation()) {
        //     drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        // }
        // drivetrain.registerTelemetry(logger::telemeterize);
        // arm.setDefaultCommand(new ArmCommand(arm, () -> ArmSetPoints.home, () ->
        // false));
        fourBar.setDefaultCommand(new FourBarCommand(fourBar, () -> FourBarConstants.fourBarHome + 0));
    }

    private void configureNamedCommands() {
        // NamedCommands.registerCommand(
        //         "shootSpeed",
        //         new ShooterCommand(
        //                 shooter,
        //                 () -> AutoAim.calculateShooterRPM(
        //                         () -> drivetrain.getState().Pose,
        //                         () -> new Translation2d(
        //                                 drivetrain.getState().speeds.vxMetersPerSecond,
        //                                 drivetrain.getState().speeds.vyMetersPerSecond)),
        //                 () -> AutoAim.calculateShooterRPM(
        //                         () -> drivetrain.getState().Pose,
        //                         () -> new Translation2d(
        //                                 drivetrain.getState().speeds.vxMetersPerSecond,
        //                                 drivetrain.getState().speeds.vyMetersPerSecond))));

        NamedCommands.registerCommand("shootSpeed", new ShooterCommand(shooter, () -> 5000.0, () -> 5000.0));

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

        NamedCommands.registerCommand("fourBarToWSR2", new FourBarCommand(fourBar, () -> AutoConstants.weirdSideRing2));
        NamedCommands.registerCommand("fourBarToWSR3", new FourBarCommand(fourBar, () -> AutoConstants.weirdSideRing3));
        NamedCommands.registerCommand("fourBarToWSR4", new FourBarCommand(fourBar, () -> AutoConstants.weirdSideRing4));

      
      
        // NamedCommands.registerCommand("fourBarToRRPShoot", new FourBarCommand(fourBar, () ->
        // AutoConstants.weirdSideRing4));

        // NamedCommands.registerCommand(
        //         "RfourBarToWSR1", new FourBarCommand(fourBar, () -> AutoConstants.red_weirdSideRing1));
        NamedCommands.registerCommand(
                "RfourBarToWSR2", new FourBarCommand(fourBar, () -> AutoConstants.red_weirdSideRing2));
        NamedCommands.registerCommand(
                "RfourBarToWSR3", new FourBarCommand(fourBar, () -> AutoConstants.red_weirdSideRing3));
        NamedCommands.registerCommand(
                "RfourBarToWSR4", new FourBarCommand(fourBar, () -> AutoConstants.red_weirdSideRing4));

        NamedCommands.registerCommand(
                "fourBarToBWSDR2", new FourBarCommand(fourBar, () -> AutoConstants.blueWeirdSideDropring2));

        NamedCommands.registerCommand(
                "RfourBarToBWSDR2", new FourBarCommand(fourBar, () -> AutoConstants.redWeirdSideDropRing2));

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

        NamedCommands.registerCommand(
                "redRECenterNote5", new FourBarCommand(fourBar, () -> AutoConstants.redRECenterNote5));

        NamedCommands.registerCommand(
                "redRECenterNote6", new FourBarCommand(fourBar, () -> AutoConstants.redRECenterNote6));

        NamedCommands.registerCommand(
                "blueRECenterNote5", new FourBarCommand(fourBar, () -> AutoConstants.blueRECenterNote5));
        NamedCommands.registerCommand(
                "blueRECenterNote6", new FourBarCommand(fourBar, () -> AutoConstants.blueRECenterNote6));

        NamedCommands.registerCommand("blueAmpSide1", new FourBarCommand(fourBar, () -> AutoConstants.blueAmpSide1));
        NamedCommands.registerCommand("blueAmpSide2", new FourBarCommand(fourBar, () -> AutoConstants.blueAmpSide2));
        NamedCommands.registerCommand("blueAmpSide3", new FourBarCommand(fourBar, () -> AutoConstants.blueAmpSide3));
        NamedCommands.registerCommand("blueAmpSide4", new FourBarCommand(fourBar, () -> AutoConstants.blueAmpSide4));

        NamedCommands.registerCommand("redAmpSide2", new FourBarCommand(fourBar, () -> AutoConstants.redAmpSide2));
        NamedCommands.registerCommand("redAmpSide3", new FourBarCommand(fourBar, () -> AutoConstants.redAmpSide3));
        NamedCommands.registerCommand("redAmpSide4", new FourBarCommand(fourBar, () -> AutoConstants.redAmpSide4));
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
                                .alongWith(
                                        new InstantCommand(() -> driverRaw.setRumble(RumbleType.kBothRumble, 0.3)))));
        driver.leftTrigger()
                .onFalse(new IntakeCommand(intake, () -> 0.0, IntakeMode.FORCEINTAKE)
                        .alongWith(new InstantCommand(() -> driverRaw.setRumble(RumbleType.kBothRumble, 0))));

        // driver shoot
        driver.rightTrigger().whileTrue(new IntakeCommand(intake, () -> 1.0, IntakeMode.FORCEINTAKE));
        driver.rightTrigger().onFalse(new IntakeCommand(intake, () -> 0.0, IntakeMode.FORCEINTAKE));

        // spit
        driver.rightBumper().whileTrue(new IntakeCommand(intake, () -> -0.4, IntakeMode.FORCEINTAKE));
        driver.rightBumper().onFalse(new IntakeCommand(intake, () -> 0.0, IntakeMode.FORCEINTAKE));

        // autoaim robot
        driver.leftBumper().whileTrue(drivetrain.applyRequest(() -> drive.withRotationalRate(
                        calculateAutoTurn(() -> AutoAim.calculateAngleToSpeaker(
                                        () -> drivetrain.getState().Pose,
                                        () -> new Translation2d(
                                                drivetrain.getState().speeds.vxMetersPerSecond,
                                                drivetrain.getState().speeds.vyMetersPerSecond))
                                .getDegrees()))
                .withVelocityX(xLimiter.calculate(-JoystickMap.JoystickPowerCalculate(driver.getRightY()) * MaxSpeed))
                .withVelocityY(
                        yLimiter.calculate(-JoystickMap.JoystickPowerCalculate(driver.getRightX()) * MaxSpeed))));

        // zero gyro
        driver.start().onTrue(new InstantCommand(() -> resetGyro()));
        // driver.a().onTrue(new InstantCommand(() -> printSpeakerDistanceAndAngle(aprilTagVision)));
        // driver.y()
        //         .whileTrue(new DriveToPoseCommand(
        //                 drivetrain,
        //                 drivetrain::getCurrentPose,
        //                 new Pose2d(14.78, 7.25, new Rotation2d(Units.degreesToRadians(-90)))));

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

        // autoaim robot
        copilot.rightTrigger().whileTrue(drivetrain.applyRequest(() -> drive.withRotationalRate(
                        calculateAutoTurn(() -> AutoAim.calculateAngleToSpeaker(
                                        () -> drivetrain.getState().Pose,
                                        () -> new Translation2d(
                                                drivetrain.getState().speeds.vxMetersPerSecond,
                                                drivetrain.getState().speeds.vyMetersPerSecond))
                                .getDegrees()))
                .withVelocityX(xLimiter.calculate(-JoystickMap.JoystickPowerCalculate(driver.getRightY()) * MaxSpeed))
                .withVelocityY(
                        yLimiter.calculate(-JoystickMap.JoystickPowerCalculate(driver.getRightX()) * MaxSpeed))));

        // 4 bar + shooter autoaim
        copilot.leftTrigger()
                .whileTrue(new ShooterCommand(
                                shooter,
                                () -> m_AutoAim.calculateShooterRPM(
                                        () -> drivetrain.getState().Pose,
                                        () -> new Translation2d(
                                                drivetrain.getState().speeds.vxMetersPerSecond,
                                                drivetrain.getState().speeds.vyMetersPerSecond)),
                                () -> m_AutoAim.calculateShooterRPM(
                                        () -> drivetrain.getState().Pose,
                                        () -> new Translation2d(
                                                drivetrain.getState().speeds.vxMetersPerSecond,
                                                drivetrain.getState().speeds.vyMetersPerSecond)))
                        .alongWith(new FourBarCommand(
                                fourBar,
                                () -> m_AutoAim.calculateFourBarPosition(
                                        () -> drivetrain.getState().Pose,
                                        () -> new Translation2d(
                                                drivetrain.getState().speeds.vxMetersPerSecond,
                                                drivetrain.getState().speeds.vyMetersPerSecond)))));

        copilot.leftTrigger().onFalse(new InstantCommand(() -> shooter.stop()));

        // copilot.leftTrigger().whileTrue(new BlinkinCommand(lights, () -> (
        //
        // drivetrain.getCurrentPose().getTranslation().getDistance(aprilTagVision.getVisionPose().getTranslation())
        //                 < DrivetrainConstants.poseSyncTolerance ?

        //         (fourBar.onTarget() ? BlinkinLightsConstants.readyToShootPattern:
        // BlinkinLightsConstants.notReadyToShootPattern):
        //                         BlinkinLightsConstants.badVisionPattern
        // )));

        lights.setDefaultCommand(new BlinkinCommand(
                lights,
                () -> copilot.leftTrigger().getAsBoolean(),
                () -> intake.getBeamBreak(),
                () -> fourBar.onTarget(),
                () -> (drivetrain
                                .getCurrentPose()
                                .getTranslation()
                                .getDistance(aprilTagVision.getVisionPose().getTranslation())
                        < DrivetrainConstants.poseSyncTolerance)));

        // copilot shoot
        copilot.rightBumper().whileTrue(new IntakeCommand(intake, () -> 1.0, IntakeMode.FORCEINTAKE));
        copilot.rightBumper().onFalse(new IntakeCommand(intake, () -> 0.0, IntakeMode.FORCEINTAKE));

        // copilot spit
        // copilot.leftBumper().whileTrue(new IntakeCommand(intake, () -> -0.4, IntakeMode.FORCEINTAKE));
        // copilot.leftBumper().onFalse(new IntakeCommand(intake, () -> 0.0, IntakeMode.FORCEINTAKE));

        // TEST COMMAND
        // copilot.back()
        //         .whileTrue(new FourBarCommand(fourBar, () -> FixedShotConstants.fourBarLong)
        //                 .alongWith(new ShooterCommand(shooter, () -> 4500.0, () -> 4800.0)));
        // copilot.back().onFalse(new InstantCommand(shooter::stop));

        // 4 bar + shooter FIXED shots

        copilot.a()
                .whileTrue(new FourBarCommand(fourBar, () -> FourBarConstants.fourBarOut)
                        .alongWith(new ShooterCommand(
                                shooter,
                                () -> FixedShotConstants.RPMPointBlank,
                                () -> FixedShotConstants.RPMPointBlank)));
        copilot.a().onFalse(new InstantCommand(shooter::stop));

        copilot.start()
                .whileTrue(new FourBarCommand(fourBar, () -> FixedShotConstants.fourBarLong)
                        .alongWith(new ShooterCommand(
                                shooter, () -> FixedShotConstants.RPMLong, () -> FixedShotConstants.RPMLong)));
        copilot.start().onFalse(new InstantCommand(shooter::stop));

        copilot.x()
                .whileTrue(new ShooterCommand(
                        shooter, () -> FixedShotConstants.RPMHome, () -> FixedShotConstants.RPMHome));
        copilot.x().onFalse(new InstantCommand(shooter::stop));

        copilot.back().whileTrue(new ShooterCommand(shooter, () -> 4000.0, () -> 4000.0));
        copilot.back().onFalse(new InstantCommand(shooter::stop));

        copilot.povLeft().and(() -> copilot.getRightY() > 0.8).onTrue(new InstantCommand(() -> m_AutoAim.decDist()));
        copilot.povLeft().and(() -> copilot.getRightY() < -0.8).onTrue(new InstantCommand(() -> m_AutoAim.incDist()));

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
        //                 new IntakeCommand(
        //                                 intake,
        //                                 () -> ((driverRaw.getLeftTriggerAxis() - 0.5) * 2),
        //                                 IntakeMode.SOFTINTAKE)
        //                         .deadlineWith(new ParallelCommandGroup(new FourBarCommand(fourBar, () ->
        // Constants.fourBarOut),
        //                         drivetrain.applyRequest(() -> drive.withVelocityX(xLimiter.calculate(0.1
        //                                     * MaxSpeed
        //                                     * Math.cos(Units.degreesToRadians(noteVisionSubsystem.getYawVal())))))
        //                             .withVelocityY(yLimiter.calculate(-0.1
        //                                     * MaxSpeed
        //                                     * Math.sin(Units.degreesToRadians(noteVisionSubsystem.getYawVal()))))
        //                             .withRotationalRate(
        //                                     calculateAutoTurn(() -> noteVisionSubsystem.getYawVal())))));
    }

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

    private void scheduleArmCommands() {

        // // Climb commands
        // climb.setDefaultCommand(new ClimbCommand(climb, () -> false, () -> false));

        // shoot grappler
        driver.a()
                .or(driver.povDown())
                .onTrue(new InstantCommand(() -> driverRaw.setRumble(RumbleType.kBothRumble, 1)))
                .onFalse(new InstantCommand(() -> driverRaw.setRumble(RumbleType.kBothRumble, 0)));

        driver.povDown()
                .and(() -> driverRaw.getXButton())
                .and(() -> Math.abs(fourBar.getFourBarAngle() - FourBarConstants.fourBarHome) > 0.1)
                .whileTrue(new WaitCommand(0.2).andThen(new GrapplerCommand(climb)));
        driver.a().whileTrue(new WinchCommand(climb).onlyIf(() -> climb.getServoOut()));

        // .whileTrue(new WinchCommand(climb, () -> false)
        //         .alongWith(new WaitCommand(ClimbConstants.rumbleWait)
        //                 .andThen(new GrapplerCommand(climb, () -> driverRaw.getAButton()))))
        // .onFalse(new WinchCommand(climb, () -> true).onlyIf(() -> climb.getServoOut()));

        // retract winch
        // driver.x().whileTrue(new WaitCommand(0.5).andThen(new WinchCommand(climb, () -> true)));
        // driver.x().onFalse(new WinchCommand(climb, () -> false));

        driver.b().whileTrue(new GripperOutCommand(arm, ArmConstants.outPowerGripper));
        driver.y().whileTrue(new GripperOutCommand(arm, -ArmConstants.outPowerGripper / 4.0));
        copilot.povUp().whileTrue(new GripperOutCommand(arm, -ArmConstants.outPowerGripper / 4.0));
        copilot.povDown().whileTrue(new GripperOutCommand(arm, ArmConstants.outPowerGripper / 4.0));
        copilot.leftBumper().whileTrue(new GripperOutCommand(arm, ArmConstants.outPowerGripper));

        // PICKUP SEQUENCE
        /* spin up flywheels/move arm to catching pos,
        spin intake to transfer
        index gripper
        pull arm outward
        return arm to home */
        copilot.b()
                .whileTrue(new ShooterCommand(
                                shooter, () -> PickupSetpoints.pickupShooterRPM, () -> PickupSetpoints.pickupShooterRPM)
                        .alongWith((new FourBarCommand(fourBar, () -> PickupSetpoints.pickupFourBar))
                                .alongWith(new ArmCommandAngles(
                                        arm, () -> PickupSetpoints.pickupElbow, () -> PickupSetpoints.pickupShoulder))
                                .raceWith((new WaitCommand(PickupSetpoints.spinUpTimeout))
                                        .andThen(new IntakeCommand(intake, () -> 1.0, IntakeMode.FORCEINTAKE)
                                                .alongWith(new WaitUntilCommand(() -> !intake.getBeamBreak())
                                                        .andThen(new WaitCommand(PickupSetpoints.intakeTimeout)
                                                                .andThen(new GripperIndexCommand(arm))))))));
        // .andThen(new ArmCommandAngles(arm, () -> PickupSetpoints.pullOutElbow, () ->
        // PickupSetpoints.pullOutShoulder).withTimeout(PickupSetpoints.pickupPullTimeout)));

        // AMP SETPOINT + MICROADJUST
        copilot.y()
                .whileTrue(new ArmCommand(
                        arm,
                        () -> AmpSetpoints.amp.plus(new Translation2d(
                                (DriverStation.getAlliance().get() == Alliance.Red ? -1 : 1)
                                        * copilot.getLeftX()
                                        * AmpSetpoints.ampMultiplierX,
                                -copilot.getLeftY() * AmpSetpoints.ampMultiplierY)),
                        () -> false));

        // TRAP COMMAND
        copilot.rightStick().onTrue(new FourBarCommand(fourBar, () -> TrapSetpoints.fourBarClimb));

        copilot.rightStick().onTrue(new ArmCommandAngles(
                                arm,
                                () -> TrapSetpoints.trapArmAngle
                                        - TrapSetpoints.trapArmDifference
                                        + (copilot.getLeftY() * TrapSetpoints.trapMicroadjust),
                                () -> TrapSetpoints.trapArmAngle
                                        + (copilot.getLeftY() * TrapSetpoints.trapMicroadjust)));

        // copilot.rightStick()
        //         .onTrue(new ArmCommandAngles(arm, () -> TrapSetpoints.climbElbow, () -> TrapSetpoints.climbShoulder));

        copilot.leftStick().onTrue(new ArmCommandAngles(
                                arm,
                                () -> TrapSetpoints.pressElbow,
                                () -> TrapSetpoints.pressShoulder
                                        + (copilot.getLeftY() * TrapSetpoints.pressMicroadjust)));

        copilot.povRight()
                .whileTrue(new ArmCommandAngles(
                        arm,
                        () -> BlockSetpoints.elbow + (copilot.getRightY() * BlockSetpoints.microadjust),
                        () -> BlockSetpoints.shoulder + (copilot.getRightY() * BlockSetpoints.microadjust)));
    }

    //     @Override
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void configureDashboard() {

        /**** Driver tab ****/

        // /**** Vision tab ****/
        // final var visionTab = Shuffleboard.getTab("Vision");

        // Pose estimation
        // drivetrain.addDashboardWidgets(visionTab);

        // final var driverTab = Shuffleboard.getTab("Driver");
        // visionTab.addBoolean(
        //         "SHOOT",
        //         () -> /*shooter.onTarget() &&*/
        //                 fourBar.onTarget()); // fourbar and shooter within tolerance of target values
        // visionTab.addBoolean(
        //         "SYNCED",
        //         () -> drivetrain
        //                         .getCurrentPose()
        //                         .getTranslation()
        //                         .getDistance(aprilTagVision.getVisionPose().getTranslation())
        //                 < DrivetrainConstants.poseSyncTolerance); // vision pose within tolerance of estimated pose
        // visionTab.addBoolean("COLLECTED", intake::getBeamBreak); // collector beambreak triggered
    }

    //     @Override
    public double calculateAutoTurn(Supplier<Double> target) {
        double currentAngle = -(gyro.getAngle() - gyroOffset);

        if (target.get() != 0) {
            targetAngle = -target.get();
            // } else if (driverRaw.getPOV() != -1) {
            //     targetAngle = -driverRaw.getPOV();
        } else { // if (Math.abs(driver.getLeftX()) >= 0.1 || Math.abs(driver.getLeftY()) >= 0.1) {
            double speed = Math.copySign(
                            Math.pow(Math.abs(driver.getLeftX()) > 0.05 ? Math.abs(driver.getLeftX()) : 0, 1.1),
                            -driver.getLeftX())
                    * 5.0;
            //     targetAngle = currentAngle;
            return zLimiter.calculate(speed);
            // targetAngle = (180.0 / Math.PI) * (Math.atan2(-driver.getLeftX(), -driver.getLeftY()));
        }

        double error = (targetAngle - currentAngle) % (360.0);

        error = error > 180.0 ? error - 360.0 : error;
        error = error < -180.0 ? error + 360.0 : error;
        targetAngle = currentAngle + error;
        SmartDashboard.putNumber("angle error deg", error);
        return Math.min(
                Math.max(
                        zLimiter.calculate(gyroPid.calculate(currentAngle, targetAngle) * MaxAngularRate),
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
