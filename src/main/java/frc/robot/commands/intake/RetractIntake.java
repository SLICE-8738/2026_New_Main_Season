package frc.robot.commands.intake;

import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;

public class RetractIntake extends Command {

  private Intake m_intake;

  private ShuffleboardTab m_ShuffleboardTab;
  private SimpleWidget m_TimerEntry;

  private Timer m_Timer;

  /**
   * Creates a new intake.
   */
  public RetractIntake (Intake intake) {
    m_intake = intake;
    m_Timer = new Timer();
    m_ShuffleboardTab = Shuffleboard.getTab("Driver");
    m_TimerEntry = m_ShuffleboardTab.add("Retract Intake Timer", 0.0);
    
    //addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Timer.start();
    m_intake.setPosition(Constants.IntakeConstants.STOWED_POSITION);
    m_intake.stopRoller();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_TimerEntry.getEntry().setDouble(m_Timer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Timer.stop();
    m_Timer.reset();
    m_intake.setBrakeMode();
    m_intake.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_Timer.get() >= 3 || m_intake.isStowed()){
      return true;
    }
    return false;
    //return m_intake.isStowed();
  }
}
