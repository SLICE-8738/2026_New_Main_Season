// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  private Shooter m_Shooter;
  private ShuffleboardTab m_ShuffleboardTab;
  private GenericEntry m_ShuffleboardAngle;
  private GenericEntry m_ShuffleboardRPM;
  private GenericEntry m_ShuffleboardDistance;
  private final CommandSwerveDrivetrain m_drivetrain;
  /** Creates a new Shoot. */
  public Shoot(Shooter shooter, CommandSwerveDrivetrain drivetrain) {
    m_Shooter = shooter;
    m_drivetrain = drivetrain;
    m_ShuffleboardTab = Shuffleboard.getTab("Shooter Tuning");
    m_ShuffleboardAngle = m_ShuffleboardTab.add("Angle: ", 12).getEntry();
    m_ShuffleboardRPM = m_ShuffleboardTab.add("RPM: ", -1200).getEntry();
    m_ShuffleboardDistance = m_ShuffleboardTab.add("Distance: ", -1.0).getEntry();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = m_Shooter.distanceFromHub();//m_drivetrain.getDistanceTo(Constants.AlignTargets.RED_HUB);
    m_ShuffleboardDistance.setDouble(distance);
    // TODO uncomment and recomment depending on which mode the shooter is in
    double angle = /*m_ShuffleboardAngle.getDouble(12);*/ Constants.ShooterConstants.SHOOTER_MAP.get(distance).hoodAngle();
    double rps = /*m_ShuffleboardRPM.getDouble(-1200);*/  Constants.ShooterConstants.SHOOTER_MAP.get(distance).rpm() / 60;
    //double angle = m_ShuffleboardAngle.getDouble(12);
    //double rps = m_ShuffleboardRPM.getDouble(-1200) / 60;
    if (distance == -1) {
      m_Shooter.pivotShooter(m_Shooter.getGoodAngle());
      m_Shooter.spinFlywheels(m_Shooter.getGoodSpeed());
    }
    else {
      m_Shooter.setGoodAngle(angle);
      m_Shooter.setGoodSpeed(rps);
      m_Shooter.pivotShooter(angle);
      m_Shooter.spinFlywheels(rps); //TODO originally negative
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
