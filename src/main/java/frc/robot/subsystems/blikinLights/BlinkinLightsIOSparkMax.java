package frc.robot.subsystems.blikinLights;

import edu.wpi.first.wpilibj.PWM;
import frc.robot.Constants.BlinkinLightsConstants;

public class BlinkinLightsIOSparkMax implements BlinkinLightsIO {

    private PWM lights;
    // private AddressableLED led;

    public BlinkinLightsIOSparkMax(int lightChannel) {
        // led = new AddressableLED(BlinkinLightsConstants.lightChannel);
        // led.start();
        // led.setLength();
        // led.setRGB();
        lights = new PWM(lightChannel);

        lights.setPulseTimeMicroseconds(2125);
        lights.setSpeed(0.67); // TODO debug value to see if this works
        // lights.setSafetyEnabled(false);
        // lights.setExpiration(1000);
        // lights.feed();
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
        lights.setSpeed(patternValue);
    }

    @Override
    @SuppressWarnings("static-access")
    public void updateInputs(BlinkintLightsIOInputs inputs) {}
}
