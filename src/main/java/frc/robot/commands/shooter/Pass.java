// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Pass extends Command {
  private Shooter m_Shooter;
  private ShuffleboardTab m_ShuffleboardTab;
  private GenericEntry m_ShuffleboardRPM;
  private SimpleWidget m_ShuffleboardDistance;
  private SimpleWidget m_ShuffleboardGoodSpeed;
  /** Creates a new Pass. */
  public Pass(Shooter shooter) {
    m_Shooter = shooter;
    m_ShuffleboardTab = Shuffleboard.getTab("Pass Tuning");
    m_ShuffleboardRPM = m_ShuffleboardTab.add("RPM: ", -1200).getEntry();
    m_ShuffleboardDistance = m_ShuffleboardTab.add("Distance: ", -1.0);
    m_ShuffleboardGoodSpeed = m_ShuffleboardTab.add("Last good Speed", m_Shooter.getGoodSpeed());
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = m_Shooter.distanceFromTrench();
    m_ShuffleboardDistance.getEntry().setDouble(distance);
    m_ShuffleboardGoodSpeed.getEntry().setDouble(m_Shooter.getGoodSpeed());
    double rpm = Constants.ShooterConstants.PASSING_MAP.get(distance).rpm(); 
    //double rpm = m_ShuffleboardRPM.getDouble(-1200);
    if (distance == -1) {
      m_Shooter.spinFlywheels(-m_Shooter.getGoodSpeed());
    }
    else {
      m_Shooter.setGoodSpeed(rpm);
      m_Shooter.spinFlywheels(rpm); 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Shooter.windDownFlywheels();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
