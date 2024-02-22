package frc.robot.util;

import edu.wpi.first.math.util.Units;

public class FourBarAngleCalculator {

    private double a = 8.5;
    private double b = 5.5;
    private double c = 6.5;
    private double d = 5.0;

    private double k1 = d / a;
    private double k2 = d / c;
    private double k3 = (Math.pow(a, 2) - Math.pow(b, 2) + Math.pow(c, 2) + Math.pow(d, 2)) / (2 * a * c);
    private double k4 = d / b;
    private double k5 = (Math.pow(c, 2) - Math.pow(d, 2) - Math.pow(a, 2) - Math.pow(b, 2)) / (2 * a * b);

    private double offset = Units.degreesToRadians(33.5);

    public double FourBarAngleCalculator(double encoderReading) {

        double theta_2 = Units.rotationsToRadians(encoderReading * (18.0 / 56.0) / 25.0) + offset;

        double D = Math.cos(theta_2) - k1 + (k4 * Math.cos(theta_2)) + k5;
        double E = -2 * Math.sin(theta_2);
        double F = k1 + (k4 - 1) * Math.cos(theta_2) + k5;

        double fourbarAngle = 2 * Math.atan((-E + Math.sqrt(Math.pow(E, 2) - 4 * D * F)) / (2 * D));
        // TODO this is the inverted angle, need to find non-inverted.
        return fourbarAngle;
    }

    public double InverseFourBarAngleCalculator(double targetAngle) {

        double encoderTarget = 0;
        return encoderTarget;
    }
}
