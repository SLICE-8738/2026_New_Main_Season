// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class ToggleIntake extends Command {

  private Intake m_intake;
  private Indexer m_indexer;

  private enum State {
    EXTENDING, INTAKING, RETRACTING, DONE
  }

  private State state;

  /**
   * Creates a new intake.
   */
  public ToggleIntake(Intake intake, Indexer indexer) {
    m_intake = intake;
    m_indexer = indexer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_intake.isStowed()) {
      state = State.EXTENDING;
      m_intake.moveIntakeToPosition(Constants.IntakeConstants.DEPLOYED_POSITION);
      m_intake.spinRoller(Constants.IntakeConstants.ROLLER_SPEED);
      m_indexer.runStageOneMotor(Constants.IndexerConstants.STAGE_ONE_INTAKE_SPEED);
    } else {
      state = State.RETRACTING;
      m_intake.moveIntakeToPosition(Constants.IntakeConstants.STOWED_POSITION);
      m_intake.spinRoller(Constants.IntakeConstants.ROLLER_RETRACT_SPEED);
      m_indexer.runStageOneMotor(Constants.IndexerConstants.STAGE_ONE_INTAKE_SPEED);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (state) {
      case EXTENDING:
        if (m_intake.isDeployed())
          state = State.INTAKING;
        break;
      case INTAKING:
        // Stays here until button is pressed again (toggleOnTrue cancels + restarts)
        m_intake.spinRoller(Constants.IntakeConstants.ROLLER_SPEED);
        break;
      case RETRACTING:
        if (m_intake.isStowed()) {
          state = State.DONE;
          m_intake.stopRoller();
          m_indexer.stopAll();
        }
        break;
      case DONE:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
