// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.*;

import java.util.concurrent.BlockingDeque;

import org.w3c.dom.events.MutationEvent;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.MathUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {

    // private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    // private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  .withDeadband(0.1).withRotationalDeadband(0.1) // Add a 10% deadband
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
            
 private final CommandXboxController joystick = new CommandXboxController(0);

  private final CommandSwerveDrivetrain m_drivetrain;
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private Field2d field;

  private double robotX;
  private double robotY;
  private double robotAngle;

  private double hubX;
  private double hubY;

  private PIDController rotationController;

  public AutoAlign(CommandSwerveDrivetrain drivetrain, XboxController controller) {
    m_drivetrain = drivetrain;

    field = new Field2d();

    robotX = m_drivetrain.getState().Pose.getX();
    robotY = m_drivetrain.getState().Pose.getY();
    robotAngle = m_drivetrain.getState().Pose.getRotation().getRadians();

    hubX = 0;
    hubY = 0;

    rotationController = new PIDController(.1, 0, 0);


  }

public double calculateAngleToHub(double robotX, double robotY, double HubPosX, double HubPosY) {
    return 1/(Math.tan((HubPosY - robotY)/(HubPosX - robotX)));
}

public double getRotation() {
    return rotationController.calculate(calculateAngleToHub(robotX, robotY, hubY, hubX), robotAngle);
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {   
    robotX = m_drivetrain.getState().Pose.getX();
    robotY = m_drivetrain.getState().Pose.getY();
    robotAngle = m_drivetrain.getState().Pose.getRotation().getRadians();

    m_drivetrain.applyRequest(() ->
    drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    );

    

  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }


  
}