// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ArmConstants.TrapSetpoints;

public class ClimbIOSparkMax implements ClimbIO {
    /** Creates a new ClimbIOSparkMax. */
    private TalonFX climbMotor;

    private Servo grapplingServo;
    private boolean servoOut = false;
    private double initialPose;

    public ClimbIOSparkMax() {
        grapplingServo = new Servo(ClimbConstants.servoPort);
        climbMotor = new TalonFX(ClimbConstants.winchPort, "canivore1");
        climbMotor.setPosition(0);
        initialPose = grapplingServo.getPosition();
        // climbMotor.setNeutralMode(NeutralModeValue.Brake);
        // climbMotor.

        TalonFXConfiguration climbConfig = new TalonFXConfiguration();
        climbConfig.Slot0 = new Slot0Configs().withKP(0.2).withKS(1);
        climbConfig.CurrentLimits =
                new CurrentLimitsConfigs().withStatorCurrentLimit(TrapSetpoints.winchAmpLimit).withStatorCurrentLimitEnable(true);
        climbConfig.MotorOutput = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);


        climbMotor.getConfigurator().apply(climbConfig);
    }

    @Override
    @SuppressWarnings("static-access")
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.climbPosition = climbMotor.getPosition().getValue();
        inputs.climbVelocity = climbMotor.getVelocity().getValue();
        inputs.climbAppliedVolts = climbMotor.getMotorVoltage().getValue();
        inputs.climbCurrentAmps = new double[] {climbMotor.getStatorCurrent().getValue()};
    }

    public void setClimbMotorPower(double climbPower) {
        if (climbMotor.getStatorCurrent().getValueAsDouble() > 100) {
            climbMotor.set(0);
        } else {
            climbMotor.set(climbPower);
        }
    }

    public double getClimbMotorPosition() {
        return climbMotor.getPosition().getValue();
    }

    public void setServoPosition(double angle) {
        grapplingServo.setAngle(angle);
        // if (angle == ClimbConstants.servoOutPos) {
            servoOut = true;
        // }
    }

    public void setServo(double power) {
        // if (power == 0) {
            // grapplingServo.setDisabled();
            // grapplingServo.setSpeed(power);
        // } else {
            grapplingServo.set(power);
            servoOut = true;
        // }
    }


    public boolean getServoOut() {
        return servoOut;
        // return Math.abs(grapplingServo.getAngle() - ClimbConstants.servoOutPos) < ClimbConstants.servoOutTolerance;
    }

    public void setClimbPosition(double position) {

        climbMotor.setControl(new PositionTorqueCurrentFOC(position));
    }
}
