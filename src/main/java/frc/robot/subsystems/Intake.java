// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.slicelibs.TalonFXPositionalSubsystem;

@SuppressWarnings("unused")

public class Intake extends TalonFXPositionalSubsystem {

  private TalonFX rotationMotor;

  /** Creates a new Intake. */
  public Intake() {
    // Initialize the Positional Subsystem and motors controlling it
    super(
      new int[] { Constants.IntakeConstants.EXTENDER_MOTOR_ID },
      new boolean[] { false },
      Constants.IntakeConstants.EXTENDER_KP, Constants.IntakeConstants.EXTENDER_KI, Constants.IntakeConstants.EXTENDER_KP, Constants.IntakeConstants.EXTENDER_KG,
      Constants.IntakeConstants.EXTENDER_RATIO,
      GravityTypeValue.Elevator_Static,
      Constants.IntakeConstants.POSITION_CONVERSION_FACTOR,
      Constants.IntakeConstants.VELOCITY_CONVERSION_FACTOR,
      Constants.CTRE_CONFIGS.extenderConfigs
    );
    setEncoderPosition(0);
    
    // Non-positional motor for spinning the roller
    rotationMotor = new TalonFX(Constants.IntakeConstants.ROTATION_MOTOR_ID);

    // Set motor configs for the roller motor
    rotationMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.rollerConfigs);

  }

  /**
   * Sets speed of rotation motor
   * @param speed speed to set the motor to (-1.0 to 1.0)
   */
  public void spinRoller(double speed) {
    rotationMotor.set(speed);
  }

  public void stopRoller() {
    rotationMotor.set(0.0);
  }
  /**
   * Moves the intake to the set position
   * @param position position to have the intake move to
   */
  public void moveIntakeToPosition(double position) {
    setPosition(position);
  }
  
  public boolean isStowed() { return (getExtenderPosition() < .127); } //5" bumper tolerance

  public boolean isDeployed() { return (getExtenderPosition() > Constants.IntakeConstants.DEPLOYED_POSITION - 0.0127); } // 0.5" tolerance

  public double getExtenderPosition(){
    return getPositions()[0];
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake extender position: ", getExtenderPosition());
    // This method will be called once per scheduler run
  }
}
