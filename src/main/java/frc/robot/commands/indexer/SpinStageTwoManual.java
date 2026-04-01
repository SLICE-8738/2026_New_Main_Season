// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinStageTwoManual extends Command {
  /** Creates a new SpinStageTwo. */
  private Indexer m_Indexer;
  private double speed;
  public SpinStageTwoManual(Indexer m_Indexer, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Indexer = m_Indexer;
    this.speed = speed;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()  { 
    m_Indexer.runStageTwoMotor(speed);
  }
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { 
    m_Indexer.runStageTwoMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
