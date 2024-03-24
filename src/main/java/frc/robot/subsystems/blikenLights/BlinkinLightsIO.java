package frc.robot.subsystems.blikenLights;

import org.littletonrobotics.junction.AutoLog;

public interface BlinkinLightsIO {

     @AutoLog
    class BlinkintLightsIOInputs {
        public double setPattern = 0.0;
    }

    default void updateInputs(BlinkintLightsIOInputs inputs) {}

    default void setLightPattern(double patternValue) {
        
    }

}
