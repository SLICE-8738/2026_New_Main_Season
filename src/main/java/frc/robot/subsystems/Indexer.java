// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {

  //class variables
  private TalonFX stageOneMotor;
  private TalonFX stageTwoMotor;

  DutyCycleOut indexerRequest = new DutyCycleOut(0).withEnableFOC(true);

  /** Creates a new Indexer. */
  public Indexer() {
    // Define the motors
    stageOneMotor = new TalonFX(Constants.IndexerConstants.STAGE_ONE_MOTOR_ID);
    stageTwoMotor = new TalonFX(Constants.IndexerConstants.STAGE_TWO_MOTOR_ID);
    
    // Set the motor configs
    stageOneMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.indexerConfigs);
    stageTwoMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.indexerConfigs);
    
  }

  /**
   * This command runs the Stage One Motor at a set speed
   * The Stage One Motor is the motor that initially moves the fuel into the Hopper after being intaken
   * @param speed the speed to set the motor to (proportional supply voltage)
   */
  public void runStageOneMotor(double speed) {
    stageOneMotor.setControl(indexerRequest.withOutput(-speed));
  }
  
  /**
   * This command runs the Stage Two Motor at a set speed
   * The Stage Two Motor is the motor that helps index the fuel inton the Shooter to be either passed or shoot at the Hub
   * @param speed the speed to set the motor to (proportional supply voltage)
   */
  public void runStageTwoMotor(double speed) {
    stageTwoMotor.setControl(indexerRequest.withOutput(-speed));
  }

  /**
   * This method completely stops ALL motors until they are called again.
   */
  public void stopAll(){
    stageOneMotor.stopMotor();
    stageTwoMotor.stopMotor();
  }

  public double getStageOneSpeed(){
    return stageOneMotor.getVelocity().getValueAsDouble();
  }

  public double getStageTwoSpeed(){
    return stageTwoMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
