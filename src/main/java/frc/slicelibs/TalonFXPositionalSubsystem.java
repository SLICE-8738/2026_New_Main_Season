// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.slicelibs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This is an extension of WPILibs SubsystemBase that implements integrated
 * functionality for positional and velocity based PID control.
 * It it intended to be used as a subclass, and the superclass can add
 * additional functionality like sensors or additional motors.
 */
public class TalonFXPositionalSubsystem extends SubsystemBase {
    private TalonFX[] motors;
    private double positionConversionFactor;
    private double velocityConversionFactor;
    private double positionTargetReference;
    private double velocityTargetReference;

    /**
     * Construct a new Positional Subsystem
     * 
     * @param ids                      an array of CAN ids for every positionally
     *                                 controlled motor in the subsystem. All motors
     *                                 should rotate together, if additional motors
     *                                 should spin seperately, add them in the
     *                                 superclass
     * @param inverted                 an array of booleans indicating whether the
     *                                 motor of the corresponding CAN id is
     *                                 inverted. This should be the same length as
     *                                 ids. true is CCW+ false is CW+.
     * @param kP                       the p gain of the subsystem (in volts per
     *                                 unit).
     * @param kI                       the i gain of the subsystem (in volts per
     *                                 unit seconds).
     * @param kD                       the d gain of the subsystem (in volts per
     *                                 units per second).
     * @param kG                       the g gain of the subsystem (in volts).
     * @param sensorToMechRatio        the ratio from the output shaft of the motor
     *                                 to the output of the mechanism. >1 is a
     *                                 reduction. This should be used for gear
     *                                 ratios, not unit conversions.
     * @param positionConversionFactor the number that, when multiplied by the
     *                                 mechanism's rotations, gives the position of
     *                                 the mechanism in desired units. This should
     *                                 be used for unit conversions, not gear
     *                                 ratios.
     * @param velocityConversionFactor the number that, when multiplied by the
     *                                 mechanism's rotations per second, gives the
     *                                 velocity of the mechanism in desired units.
     *                                 This should be used for unit conversions, not
     *                                 gear ratios.
     * @param motorConfigs             the configs for the provided motor. The Slot0
     *                                 PID controller, InvertedValue, and
     *                                 SensorToMechanismRatio will be overriden by
     *                                 other parameters.
     */
    public TalonFXPositionalSubsystem(int[] ids, boolean[] inverted, double kP, double kI, double kD, double kG,
            double sensorToMechRatio,
            GravityTypeValue gravityType, double positionConversionFactor, double velocityConversionFactor,
            TalonFXConfiguration motorConfigs) {
        if (ids.length != inverted.length)
            throw new IllegalArgumentException("ids and inverted must be the same length");

        motors = new TalonFX[ids.length];
        for (int i = 0; i < ids.length; i++) {
            TalonFXConfiguration configs = motorConfigs;

            if (inverted[i]) {
                configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            } else {
                configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            }
            configs.Slot0.kP = kP * sensorToMechRatio;
            configs.Slot0.kI = kI * sensorToMechRatio;
            configs.Slot0.kD = kD * sensorToMechRatio;
            configs.Slot0.kG = kG;
            configs.Slot0.GravityType = gravityType;
            configs.Feedback.SensorToMechanismRatio = sensorToMechRatio;

            motors[i] = new TalonFX(ids[i]);
            motors[i].getConfigurator().apply(configs);
        }

        this.positionConversionFactor = positionConversionFactor;
        this.velocityConversionFactor = velocityConversionFactor;
        positionTargetReference = 0;
        velocityTargetReference = 0;
    }

    /**
     * Directly sets the speed of all positional motors via duty cycle control
     * 
     * @param speed a value from -1 to 1
     */
    public void set(double speed) {
        for (TalonFX motor : motors) {
            motor.set(speed);
        }
    }

    /**
     * Directly sets the voltage being supplied to all positional motors
     * 
     * @param voltage volts supplied to each motor (this should generally be between
     *                -12 and 12)
     */
    public void setVoltage(double volts) {
        for (TalonFX motor : motors) {
            motor.setVoltage(volts);
        }
    }

    /**
     * Sets the target speed of the subsystem, which it will approach using its
     * internal PID controller.
     * 
     * @param velocity a velocity value in the units defined by the
     *                 velocityConversionFactor
     */
    public void setVelocity(double velocity) {
        VelocityVoltage request = new VelocityVoltage(0).withSlot(0);

        for (TalonFX motor : motors) {
            motor.setControl(request.withVelocity(velocity / velocityConversionFactor));
        }
        velocityTargetReference = velocity;
    }

    /**
     * Sets the target position of the subsystem, which it will approach using its
     * internal PID controller.
     * 
     * @param velocity a value in the units defined by the positionConversionFactor
     */
    public void setPosition(double position) {
        PositionVoltage request = new PositionVoltage(0).withSlot(0);
        for (TalonFX motor : motors) {
            motor.setControl(request.withPosition(position / positionConversionFactor));
        }

        positionTargetReference = position;
    }

    /**
     * Changes the encoders of all positional motors to read that they are at the
     * given position
     * 
     * @param position a value in the units defined by the positionConversionFactor
     */
    public void setEncoderPosition(double position) {
        for (TalonFX motor : motors) {
            motor.getConfigurator().setPosition(position / positionConversionFactor);
        }
    }

    /**
     * Returns an array of the positions of all motors.
     * 
     * @return the subsystems's position, in the units defined by the
     *         positionConversionFactor
     */
    public double[] getPositions() {
        double[] position = new double[motors.length];
        for (int i = 0; i < motors.length; i++) {
            position[i] = motors[i].getPosition().getValueAsDouble() * positionConversionFactor;
        }
        return position;
    }

    /**
     * Returns an array the stator currents of all motors.
     * 
     * @return the subsystems stator current values.
     * 
     */
    public double[] getStatorCurrents() {
        double[] currents = new double[motors.length];

        for (int i = 0; i < motors.length; i++) {
            currents[i] = motors[i].getStatorCurrent().getValueAsDouble();
        }
        return currents;
    }

    public double getTargetPosition() {
        return positionTargetReference;
    }

    public double getTargetVelocity() {
        return velocityTargetReference;
    }

    /**
     * Returns the average velocity reading across all positional motors.
     * 
     * @return the subsystems's velocity, in the units defined by the
     *         velocityConversionFactor
     */
    public double[] getVelocity() {
        double[] velocity = new double[motors.length];
        for (int i = 0; i < motors.length; i++) {
            velocity[i] = motors[i].getVelocity().getValueAsDouble() * velocityConversionFactor;
        }
        return velocity;
    }

    /**
     * Returns whether the subsystem's position or velocity (depending on which
     * control type was used last) is close to the target value
     * 
     * @param threshold the maximum acceptable error, in the units defined by either
     *                  positionConversionFactor or velocityConversionFactor
     * @return true if the subsystem is near its target, false if otherwise (or if
     *         the subsystem was last controlled using set() or setVoltage())
     */
    public boolean atTarget(double threshold) {
        boolean[] atTarget = new boolean[motors.length];
        if (motors[0].getAppliedControl().getClass() == PositionVoltage.class) {
            for (int i = 0; i < motors.length; i++) {
                atTarget[i] = Math
                        .abs(motors[i].getPosition().getValueAsDouble()
                                - (positionTargetReference / positionConversionFactor)) < (threshold
                                        / positionConversionFactor);
            }
        } else if (motors[0].getAppliedControl().getClass() == VelocityVoltage.class) {
            for (int i = 0; i < motors.length; i++) {
                atTarget[i] = Math
                        .abs(motors[i].getVelocity().getValueAsDouble()
                                - (velocityTargetReference / velocityConversionFactor)) < (threshold
                                        / velocityConversionFactor);
            }
        }

        for (boolean target : atTarget) {
            if (!target) {
                return false;
            }
        }
        return true;
    }

    /**
     * @return the last target position this mechanism was trying to reach.
     */
    public double getPositionTargetReference() {
        return positionTargetReference;
    }

}