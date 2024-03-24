package frc.robot.subsystems.blikenLights;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.BlinkinLightsConstants;

public class BlinkinLightsIOSparkMax implements BlinkinLightsIO {
    
    private Spark blinkenLights;


    public BlinkinLightsIOSparkMax() {
        blinkenLights = new Spark(BlinkinLightsConstants.lightChannel);
    }

    public void setLightPattern(double patternValue) {
        blinkenLights.set(patternValue);
    }

    @Override
    @SuppressWarnings("static-access")
    public void updateInputs(BlinkintLightsIOInputs inputs) {
        
    }

    
}
