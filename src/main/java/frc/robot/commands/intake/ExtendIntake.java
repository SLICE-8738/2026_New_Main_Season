package frc.robot.commands.intake;

import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ExtendIntake extends Command {

  private Intake m_intake;
  private Timer m_Timer;

  /**
   * Creates a new intake.
   */
  public ExtendIntake (Intake intake) {
    m_intake = intake;
    m_Timer = new Timer();
    //addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Timer.reset();
    m_Timer.start();
    m_intake.setPosition(Constants.IntakeConstants.DEPLOYED_POSITION);
    m_intake.spinRoller(Constants.IntakeConstants.ROLLER_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setCoastMode();
    m_intake.stopMotors();
    m_Timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_Timer.get() >= 1.5 || m_intake.isDeployed()){
      return true;
    }
    return false;
    //return m_intake.isDeployed();
  }
}
