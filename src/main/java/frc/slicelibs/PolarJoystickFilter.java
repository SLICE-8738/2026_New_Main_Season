package frc.slicelibs;

import frc.slicelibs.configs.JoystickFilterConfig;

public class PolarJoystickFilter {

    private double lastInput;
    private double lastThetaValue;
    private final double deadzone, smoothing, exponent, exponentPercent;

    public PolarJoystickFilter(JoystickFilterConfig config) {
        this.deadzone = config.deadzone;
        this.smoothing = config.smoothing;
        this.exponent = config.exponent;
        this.exponentPercent = config.exponentPercent;
        lastInput = 0;
    }

    public double[] toPolar(double rawX, double rawY) {
        if (rawX == 0 && rawY == 0) return new double[] {0, 0};

        double[] polar = {Math.atan2(rawY, rawX), Math.sqrt(rawX * rawX + rawY * rawY)};
        lastThetaValue = polar[0];
        if (polar[1] > 1) polar[1] = 1;
        return polar;
    }

    public double[] withDead(double[] polar) {
        return polar[1] < deadzone ? new double[] {lastThetaValue, 0} : polar;
    }

    public double withCurve(double raw) {
        double curved = Math.copySign(exponentPercent * Math.pow(Math.abs(raw), exponent), raw);
        return curved + (1 - exponentPercent) * raw;
    }

    public double[] filter(double rawX, double rawY) {
        double[] filtered = withDead(toPolar(rawX, rawY));
        filtered[1] = withCurve(filtered[1]);
        filtered[1] = smoothing * lastInput + (1 - smoothing) * filtered[1];
        lastInput = filtered[1];
        double[] signal = withDead(filtered);
        return new double[] {Math.cos(signal[0]) * signal[1], Math.sin(signal[0]) * signal[1]};
    }
}