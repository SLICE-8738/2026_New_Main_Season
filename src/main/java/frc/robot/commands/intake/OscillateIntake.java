// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class OscillateIntake extends Command {

  private Intake m_intake;
  private double targetPosition;
  private enum State { EXTENDING, RETRACTING }
  private State state;

  /**
   * Creates a new intake.
   */
  public OscillateIntake (Intake intake) {
    m_intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPosition = m_intake.getExtenderPosition();
    state = m_intake.isDeployed() ? State.EXTENDING : State.RETRACTING;
    m_intake.spinRoller(Constants.IntakeConstants.ROLLER_RETRACT_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      switch (state) {
      case EXTENDING:
        m_intake.moveIntakeToPosition(targetPosition);
        if (m_intake.getExtenderPosition() > targetPosition - .0075 && m_intake.getExtenderPosition() < targetPosition + .0075)
            targetPosition = targetPosition <= Constants.IntakeConstants.STOWED_POSITION ? Constants.IntakeConstants.STOWED_POSITION + Constants.IntakeConstants.OSCILLATION_AMOUNT : targetPosition - (Constants.IntakeConstants.OSCILLATION_AMOUNT + Constants.IntakeConstants.OSCILLATION_DIFF);
            state = State.RETRACTING;
        break;
      case RETRACTING:
        m_intake.moveIntakeToPosition(targetPosition);
        if (m_intake.getExtenderPosition() > targetPosition - .0075 && m_intake.getExtenderPosition() < targetPosition + .0075)
            targetPosition += (Constants.IntakeConstants.OSCILLATION_AMOUNT);
            state = State.EXTENDING;
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopRoller();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
