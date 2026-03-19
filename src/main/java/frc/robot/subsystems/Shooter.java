// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ShooterConstants.FullShooterParams;
import frc.slicelibs.TalonFXPositionalSubsystem;

public class Shooter extends TalonFXPositionalSubsystem {

  private TalonFX leftShooterMotor, rightShooterMotor;

  private double targetSpeed, targetPosition;

  /** Creates a new Shooter. */
  public Shooter() {
    
    super(
      new int[] { Constants.ShooterConstants.PIVOT_MOTOR_ID },
      new boolean[] { true },
      Constants.ShooterConstants.AIM_KP, Constants.ShooterConstants.AIM_KI, Constants.ShooterConstants.AIM_KD, Constants.ShooterConstants.AIM_KG,
      Constants.ShooterConstants.PIVOT_GEAR_RATIO,
      GravityTypeValue.Arm_Cosine,
      Constants.ShooterConstants.POSITION_CONVERSION_FACTOR,
      Constants.ShooterConstants.VELOCITY_CONVERSION_FACTOR,
      Constants.CTRE_CONFIGS.pivotConfigs
    );

    setEncoderPosition(Constants.ShooterConstants.SHOOTER_STOW); // 12 degree from ground stow angle

    // Define the motors for spinning the flywheels.
    leftShooterMotor = new TalonFX(Constants.ShooterConstants.LEFT_SHOOTER_MOTOR_ID);
    rightShooterMotor = new TalonFX(Constants.ShooterConstants.RIGHT_SHOOTER_MOTOR_ID);

    // Set the motor configs.
    leftShooterMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.shooterConfigs);
    rightShooterMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.shooterConfigs);
  }

  public void windDownFlywheels(){
    leftShooterMotor.set(0);
    rightShooterMotor.set(0);
  }

  /**
   * Calculates the distance from the hub.
   * @return Distance from the hub in feet.
   */
  public static double distanceFromHub(){
    double distance = -1;
    if (!LimelightHelpers.getTV("limelight-shooter")){
      return distance; // Invalid distance
    }

    double offsetAngleVertical = LimelightHelpers.getTY("limelight-shooter");
    double angleToGoal = Math.toRadians(Constants.ShooterConstants.LIMELIGHT_ANGLE + offsetAngleVertical);

    distance = (Constants.FieldConstants.HUB_APRILTAG_HEIGHT - Constants.ShooterConstants.LIMELIGHT_HEIGHT) / Math.tan(angleToGoal);

    return Math.abs(distance);
  }

  /**
   * Sets the flywheels to spin at a certain speed.
   * @param speed The velocity in rotations per second
   */
  public void spinFlywheels(double velocity){
    VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0).withFeedForward(.1).withEnableFOC(true);

    leftShooterMotor.setControl(velocityRequest.withVelocity(velocity  * ( 28.0 / 20.0 )));
    rightShooterMotor.setControl(velocityRequest.withVelocity(-velocity  * ( 28.0 / 20.0 )));
  }

  /**
   * Move the shooter to a position in degrees.
   * @param angle The angle to set in degrees
   */
  public void pivotShooter(double angle) {
    setPosition(angle);
  }

  public double getHorizontalVelocity(double distance) {
    FullShooterParams params = Constants.ShooterConstants.SHOOTER_MAP.get(distance);
    return distance / params.tof();
  }

  public void calculateShot(double distance, double requiredVelocity) {
    FullShooterParams baseline = Constants.ShooterConstants.SHOOTER_MAP.get(distance);
    double baselineVelocity = distance / baseline.tof();
    double velocityRatio = requiredVelocity / baselineVelocity;

    // Split the correction: sqrt gives equal "contribution" from each
    double rpmFactor = Math.sqrt(velocityRatio);
    double hoodFactor = Math.sqrt(velocityRatio);

    // Apply RPM scaling
    double adjustedRpm = baseline.rpm() * rpmFactor;

    // Apply hood adjustment (changes horizontal component)
    double totalVelocity = baselineVelocity / Math.cos(Math.toRadians(baseline.hoodAngle()));
    double targetHorizFromHood = baselineVelocity * hoodFactor;
    double ratio = MathUtil.clamp(targetHorizFromHood / totalVelocity, 0.0, 1.0);
    double adjustedHood = Math.toDegrees(Math.acos(ratio));

    targetSpeed = adjustedRpm;
    targetPosition = adjustedHood;
  }

  public double getFlywheelSpeed() {
    return (leftShooterMotor.getVelocity().getValueAsDouble() + rightShooterMotor.getVelocity().getValueAsDouble()) / 2;
  }

  public double getPivotPosition() {
    return getPositions()[0];
  }

  
  public boolean atTargetSpeed() {
    return Math.abs(targetSpeed - getFlywheelSpeed()) <= Constants.ShooterConstants.FLYWHEEL_RPM_ACCEPTABLE_ERROR;
  }

  public boolean atTargetPosition() {
    return Math.abs(targetPosition - getPivotPosition()) <= (Constants.ShooterConstants.VERTICAL_AIM_ACCEPTABLE_ERROR * (Math.PI / 180));
  }

  // Determines if the hub is active
  public boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }

  // Determines if the hub is 4 seconds from active
  public boolean isHubAlmostActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if (matchTime > (130 - Constants.FieldConstants.SPEED_SHOOTER_AT)) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > (105 - Constants.FieldConstants.SPEED_SHOOTER_AT)) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > (80 - Constants.FieldConstants.SPEED_SHOOTER_AT)) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > (55 - Constants.FieldConstants.SPEED_SHOOTER_AT)) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > (30 - Constants.FieldConstants.SPEED_SHOOTER_AT)) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }

  public double getLeftVelocity(){
    return leftShooterMotor.getVelocity().getValueAsDouble();
  }

  public double getRightVelocity(){
    return rightShooterMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Shoot Velocity", getLeftVelocity());
    SmartDashboard.putNumber("Right Shoot Velocity", getRightVelocity());
    SmartDashboard.putNumber("Pivot shooter angle: ", getPivotPosition());
    SmartDashboard.putNumber("Distance from Hub: ", distanceFromHub());

    // This method will be called once per scheduler run
  }
}
