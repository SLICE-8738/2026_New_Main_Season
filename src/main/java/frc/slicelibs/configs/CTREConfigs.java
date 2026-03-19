// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.slicelibs.configs;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants;


public class CTREConfigs {

    public final TalonFXConfiguration m_swerveDriveConfigs = new TalonFXConfiguration();
    public final TalonFXConfiguration m_swerveTurnConfigs = new TalonFXConfiguration();
    public final CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public final TalonFXConfiguration shooterConfigs = new TalonFXConfiguration();
    public final TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();

    public final TalonFXConfiguration extenderConfigs = new TalonFXConfiguration();
    public final TalonFXConfiguration rollerConfigs = new TalonFXConfiguration();

    public final TalonFXConfiguration indexerConfigs = new TalonFXConfiguration();
    public final TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();

    public CTREConfigs() {
        configureSwerve();
        configureShooter();
        configureIntake();
        configureIndexer();
    }

    private void configureSwerve() {
        // CANcoder config
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.DriveConstants.ABSOLUTE_ENCODER_INVERT;
        swerveCANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

        // Drive motor config
        m_swerveDriveConfigs.Slot0.kP = Constants.DriveConstants.DRIVE_KP;
        m_swerveDriveConfigs.Slot0.kI = Constants.DriveConstants.DRIVE_KI;
        m_swerveDriveConfigs.Slot0.kD = Constants.DriveConstants.DRIVE_KD;
        m_swerveDriveConfigs.Slot0.kS = Constants.DriveConstants.DRIVE_KS;
        m_swerveDriveConfigs.Slot0.kV = Constants.DriveConstants.DRIVE_KV;
        m_swerveDriveConfigs.CurrentLimits.StatorCurrentLimit = Constants.DriveConstants.DRIVE_STATOR_CURRENT_LIMIT;
        m_swerveDriveConfigs.CurrentLimits.SupplyCurrentLimit = Constants.DriveConstants.DRIVE_SUPPLY_CURRENT_LIMIT;
        m_swerveDriveConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        m_swerveDriveConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Turn motor config
        m_swerveTurnConfigs.Slot0.kP = Constants.DriveConstants.TURN_KP;
        m_swerveTurnConfigs.Slot0.kI = Constants.DriveConstants.TURN_KI;
        m_swerveTurnConfigs.Slot0.kD = Constants.DriveConstants.TURN_KD;
        m_swerveTurnConfigs.Feedback.SensorToMechanismRatio = Constants.DriveConstants.ANGLE_GEAR_RATIO;
        m_swerveTurnConfigs.ClosedLoopGeneral.ContinuousWrap = true;
        m_swerveTurnConfigs.CurrentLimits.StatorCurrentLimit = Constants.DriveConstants.TURN_STATOR_CURRENT_LIMIT;
        m_swerveTurnConfigs.CurrentLimits.SupplyCurrentLimit = Constants.DriveConstants.TURN_SUPPLY_CURRENT_LIMIT;
        m_swerveTurnConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        m_swerveTurnConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;


        /*

        WHY IS THIS HERE LOLLLL

        //////////////////////////
        /// Shooter Configs ////
        //////////////////////////

        var shooterPID = shooterConfigs.Slot0;
        shooterPID.kP = Constants.ShooterConstants.FLYWHEEL_KP;
        shooterPID.kI = Constants.ShooterConstants.FLYWHEEL_KI;
        shooterPID.kD = Constants.ShooterConstants.FLYWHEEL_KD;
        //shooterPID.kS =
        //shooterPID.kV = 510; //TODO tune


        var shooterRightPID = shooterConfigs.Slot0;
        shooterRightPID.kP = Constants.ShooterConstants.FLYWHEEL_KP;
        shooterRightPID.kI = Constants.ShooterConstants.FLYWHEEL_KI;
        shooterRightPID.kD = Constants.ShooterConstants.FLYWHEEL_KD;

        var shooterRight = shooterConfigs.MotorOutput;
        shooterRight.Inverted = InvertedValue.Clockwise_Positive;

        //////////////////////
        /// Intake Configs ///
        //////////////////////
        
        var intakePID = intakeConfigs.Slot0;
        intakePID.kP = 0.2;
        intakePID.kI = 0;
        intakePID.kD = 0;

        var intakeOutput = intakeConfigs.MotorOutput;
        intakeOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        */

    }

    private void configureShooter() {
        // Flywheel motor configurations
        shooterConfigs.Slot0.kP = Constants.ShooterConstants.FLYWHEEL_KP;
        shooterConfigs.Slot0.kI = Constants.ShooterConstants.FLYWHEEL_KI;
        shooterConfigs.Slot0.kD = Constants.ShooterConstants.FLYWHEEL_KD;
        shooterConfigs.Slot0.kS = Constants.ShooterConstants.FLYWHEEL_KS;
        shooterConfigs.Slot0.kV = Constants.ShooterConstants.FLYWHEEL_KV;
        shooterConfigs.Feedback.SensorToMechanismRatio = Constants.ShooterConstants.FLYWHEEL_GEAR_RATIO;
        shooterConfigs.CurrentLimits.StatorCurrentLimit = Constants.ShooterConstants.FLYWHEEL_STATOR_CURRENT_LIMIT;
        shooterConfigs.CurrentLimits.SupplyCurrentLimit = Constants.ShooterConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT;
        shooterConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Pivot motor configurations
        pivotConfigs.Slot0.kP = Constants.ShooterConstants.AIM_KP;
        pivotConfigs.Slot0.kI = Constants.ShooterConstants.AIM_KI;
        pivotConfigs.Slot0.kD = Constants.ShooterConstants.AIM_KD;
        pivotConfigs.Feedback.SensorToMechanismRatio = Constants.ShooterConstants.PIVOT_GEAR_RATIO;
    }

    private void configureIntake() {
        // Extender motor configurations
        extenderConfigs.Slot0.kP = Constants.IntakeConstants.EXTENDER_KP;
        extenderConfigs.Slot0.kI = Constants.IntakeConstants.EXTENDER_KI;
        extenderConfigs.Slot0.kD = Constants.IntakeConstants.EXTENDER_KD;
        extenderConfigs.Slot0.kG = Constants.IntakeConstants.EXTENDER_KG;
        extenderConfigs.Feedback.SensorToMechanismRatio = Constants.IntakeConstants.EXTENDER_RATIO;
        extenderConfigs.CurrentLimits.StatorCurrentLimit = Constants.IntakeConstants.EXTENDER_STATOR_CURRENT_LIMIT;
        extenderConfigs.CurrentLimits.SupplyCurrentLimit = Constants.IntakeConstants.EXTENDER_SUPPLY_CURRENT_LIMIT;

        // Roller motor configurations
        rollerConfigs.Feedback.SensorToMechanismRatio = Constants.IntakeConstants.ROLLER_GEAR_RATIO;
        rollerConfigs.CurrentLimits.StatorCurrentLimit = Constants.IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT;
        rollerConfigs.CurrentLimits.SupplyCurrentLimit = Constants.IntakeConstants.ROLLER_SUPPLY_CURRENT_LIMIT;
    }

    private void configureIndexer() {
        // Indexer motors configurations
        indexerConfigs.CurrentLimits.StatorCurrentLimit = Constants.IndexerConstants.INDEXER_STATOR_CURRENT_LIMIT;
        indexerConfigs.CurrentLimits.SupplyCurrentLimit = Constants.IndexerConstants.INDEXER_SUPPLY_CURRENT_LIMIT;
    }

}