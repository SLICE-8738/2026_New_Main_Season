// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class Spintake extends Command {

  private Intake m_intake;

    /**
   * Creates a new intake.
   */
  public Spintake(Intake intake) {
    m_intake = intake;
    //addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_intake.spinRoller(Constants.IntakeConstants.ROLLER_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

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
