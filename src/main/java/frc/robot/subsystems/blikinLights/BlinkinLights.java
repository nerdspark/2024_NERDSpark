package frc.robot.subsystems.blikinLights;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class BlinkinLights extends SubsystemBase {
    private final BlinkinLightsIO io;

    private final BlinkintLightsIOInputsAutoLogged inputs = new BlinkintLightsIOInputsAutoLogged();

    public BlinkinLights(BlinkinLightsIO blinkenLightsIO) {
        this.io = blinkenLightsIO;
    }

    public void setLightPattern(double patternValue) {
        io.setLightPattern(patternValue);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Blinkin Lights", inputs);
    }
}
