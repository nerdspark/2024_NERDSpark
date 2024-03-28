package frc.robot.subsystems.blikinLights;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.BlinkinLightsConstants;

public class BlinkinLightsIOSparkMax implements BlinkinLightsIO {

    private Spark lights;
    // private AddressableLED led;

    public BlinkinLightsIOSparkMax() {
        // led = new AddressableLED(BlinkinLightsConstants.lightChannel);
        // led.start();
        // led.setLength();
        // led.setRGB();
        lights = new Spark(BlinkinLightsConstants.lightChannel);
        lights.setSafetyEnabled(false);
        lights.setExpiration(1000);
        lights.feed();
        // lights.check();
        // lights.getExpiration();
        // lights.isSafetyEnabled();
        // lights.initSendable(null);
        // lights.checkMotors();
    }


    public void setLightPattern(double patternValue) {
        // if (!lights.isAlive()) {
            // lights.feed();
        // }
        lights.set(patternValue);
    }

    @Override
    @SuppressWarnings("static-access")
    public void updateInputs(BlinkintLightsIOInputs inputs) {}
}
