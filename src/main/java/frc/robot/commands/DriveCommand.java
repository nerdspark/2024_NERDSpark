// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.JoystickMap;
import java.util.function.Supplier;

public class DriveCommand extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private double MaxSpeed = 6.0; // 6 meters per second desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    private SlewRateLimiter xLimiter = new SlewRateLimiter(3);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(3);
    private SlewRateLimiter zLimiter = new SlewRateLimiter(10);
    private double targetAngle = 0.0;
    private final Pigeon2 gyro = new Pigeon2(RobotMap.pigeonID, "canivore1");
    private PIDController gyroPid =
            new PIDController(DrivetrainConstants.gyroP, DrivetrainConstants.gyroI, DrivetrainConstants.gyroD);
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
    private Supplier<Double> LeftX, LeftY, RightX, RightY;
    private Supplier<Integer> Dpad;
    /** Creates a new DriveCommand. */
    public DriveCommand(
            CommandSwerveDrivetrain drivetrain,
            Supplier<Double> LeftX,
            Supplier<Double> RightX,
            Supplier<Double> LeftY,
            Supplier<Double> RightY,
            Supplier<Integer> Dpad) {
        this.drivetrain = drivetrain;
        this.LeftX = LeftX;
        this.LeftY = LeftY;
        this.RightX = RightX;
        this.RightY = RightY;
        this.Dpad = Dpad;

        addRequirements(drivetrain);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drivetrain.applyRequest(
                () -> drive.withVelocityX(xLimiter.calculate(
                                -JoystickMap.JoystickPowerCalculate(RightY.get()) * MaxSpeed)) // Drive forward with
                        // negative Y (forward)
                        .withVelocityY(yLimiter.calculate(-JoystickMap.JoystickPowerCalculate(RightX.get())
                                * MaxSpeed)) // Drive left with negative X (left)
                        .withRotationalRate(zLimiter.calculate(calculateAutoTurn(() -> 0.0)
                                * MaxAngularRate)) // Drive counterclockwise with negative X (left)
                );
    }

    public double calculateAutoTurn(Supplier<Double> target) {
        double currentAngle = -gyro.getAngle();

        if (target.get() != 0) {
            targetAngle = -target.get();
        } else {
            if (Dpad.get() != -1) {
                targetAngle = -Dpad.get();
            } else if (Math.abs(LeftX.get()) >= 0.1 || Math.abs(LeftY.get()) >= 0.1) {
                targetAngle = (180.0 / Math.PI) * (Math.atan2(-LeftX.get(), -LeftY.get()));
            }
        }

        double error = (targetAngle - currentAngle) % (360.0);

        error = error > 180.0 ? error - 360.0 : error;
        error = error < -180.0 ? error + 360.0 : error;
        targetAngle = currentAngle + error;
        return gyroPid.calculate(currentAngle, targetAngle);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
