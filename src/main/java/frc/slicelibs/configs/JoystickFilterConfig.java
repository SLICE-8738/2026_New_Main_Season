package frc.slicelibs.configs;

public class JoystickFilterConfig {
    public final double deadzone, smoothing, exponent, exponentPercent;

    public JoystickFilterConfig(double deadzone, double smoothing, double exponent, double exponentPercent) {
        this.deadzone = deadzone;
        this.smoothing = smoothing;
        this.exponent = exponent;
        this.exponentPercent = exponentPercent;
    }

    public JoystickFilterConfig(double deadzone, double smoothing) {
        this(deadzone, smoothing, 1, 1);
    }

    public JoystickFilterConfig(double deadzone) {
        this(deadzone, 0, 1, 1);
    }
}