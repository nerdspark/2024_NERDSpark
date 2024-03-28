// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.BlinkinLightsConstants;
import frc.robot.subsystems.blikinLights.BlinkinLights;
import java.util.function.Supplier;

public class BlinkinCommand extends Command {

    private BlinkinLights lights;
    private double color;
    private Supplier<Boolean> beambreakGood, aimGood, visionGood, aiming;

    /** Creates a new blinkinCommand. */
    public BlinkinCommand(
            BlinkinLights lights,
            Supplier<Boolean> aiming,
            Supplier<Boolean> beambreakGood,
            Supplier<Boolean> aimGood,
            Supplier<Boolean> visionGood) {
        this.lights = lights;
        this.beambreakGood = beambreakGood;
        this.aimGood = aimGood;
        this.visionGood = visionGood;
        this.aiming = aiming;
        addRequirements(lights);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        color = !beambreakGood.get()
                ? BlinkinLightsConstants.doesNotHaveNotePattern
                : (!aiming.get()
                        ? BlinkinLightsConstants.hasNotePattern
                        : (!visionGood.get()
                                ? BlinkinLightsConstants.badVisionPattern
                                : (aimGood.get()
                                        ? BlinkinLightsConstants.readyToShootPattern
                                        : BlinkinLightsConstants.notReadyToShootPattern)));
        lights.setLightPattern(color);
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
