// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {

  //class variables
  private TalonFX stageOneMotor;
  private TalonFX stageTwoMotor;

  /** Creates a new Indexer. */
  public Indexer() {
    // Define the motors
    stageOneMotor = new TalonFX(Constants.IndexerConstants.STAGE_ONE_MOTOR_ID);
    stageTwoMotor = new TalonFX(Constants.IndexerConstants.STAGE_TWO_MOTOR_ID);
    
    // Set the motor configs
    stageOneMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.indexerConfigs);
    stageTwoMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.indexerConfigs);

  }

  public void runStageOneMotor(double speed) {
    stageOneMotor.set(speed);
  }
   
  public void runStageTwoMotor(double speed) {
    stageTwoMotor.set(speed);
  }

  public void stopAll(){
    stageOneMotor.set(0.0);
    stageTwoMotor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
