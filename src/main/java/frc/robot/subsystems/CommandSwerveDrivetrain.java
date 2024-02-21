package frc.robot.subsystems;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.util.AutoAimMath;
import frc.robot.util.FieldConstants;
import frc.robot.util.VisionHelpers;
import frc.robot.util.VisionHelpers.TimestampedVisionUpdate;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final Field2d field2d = new Field2d();
    private OriginPosition originPosition = kBlueAllianceWallRightSide;

    // private Pose2d targetPoseSpeaker = AllianceFlipUtil.apply(speakerConstants.speakerLocBlue);
    // private Translation2d targetPoseSpeaker =
    // AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.getTranslation());
    private Translation2d targetPoseSpeaker = FieldConstants.Speaker.centerSpeakerOpening.getTranslation();
    private boolean targetFollow = false;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants driveTrainConstants,
            double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
    }

    public Optional<Rotation2d> getRotationTargetOverride() {
        // Some condition that should decide if we want to override rotation
        if (targetFollow) {
            // Return an optional containing the rotation override (this should be a field relative rotation)
            return Optional.of(AutoAimMath.getAutoAimCalcRobot(() -> this.getState().Pose, targetPoseSpeaker));
        } else {
            // return an empty optional when we don't want to override the path's rotation
            return Optional.empty();
        }
    }

    private void configurePathPlanner() {
        double driveBaseRadius = Units.inchesToMeters(13.25);
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        // this.seedFieldRelative(new Pose2d(1.5,6.4, new Rotation2d()));

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose,
                this::seedFieldRelative,
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)),
                new HolonomicPathFollowerConfig(
                        new PIDConstants(10, 0, 0),
                        new PIDConstants(10, 0, 0),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                // flips the path if on red alliance (do we want this? remove if we're making red-specific paths)
                () -> DriverStation.getAlliance()
                        .filter(value -> value == DriverStation.Alliance.Red)
                        .isPresent(),
                this);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        SignalLogger.writeDoubleArray("Odometry", new double[] {
            this.getState().Pose.getX(),
            this.getState().Pose.getY(),
            this.getState().Pose.getRotation().getDegrees()
        });
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds vision data to the pose esimation.
     *
     * @param visionData The vision data to add.
     */
    public void addVisionData(List<TimestampedVisionUpdate> visionData) {
        visionData.forEach(visionUpdate ->
                addVisionMeasurement(visionUpdate.pose(), visionUpdate.timestamp(), visionUpdate.stdDevs()));
    }

    public void addDashboardWidgets(ShuffleboardTab tab) {
        tab.add("Field", field2d).withPosition(0, 0).withSize(6, 4);
        tab.addString("Pose", this::getFomattedPose).withPosition(6, 2).withSize(2, 1);
    }

    private String getFomattedPose() {
        var pose = this.getState().Pose;
        return String.format(
                "(%.3f, %.3f) %.2f degrees",
                pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }

    @Override
    public void periodic() {
        // Set the pose on the dashboard
        var dashboardPose = this.getState().Pose;
        if (originPosition == kRedAllianceWallRightSide) {
            // Flip the pose when red, since the dashboard field photo cannot be rotated
            dashboardPose = VisionHelpers.flipAlliance(dashboardPose);
        }
        field2d.setRobotPose(dashboardPose);
    }
}
