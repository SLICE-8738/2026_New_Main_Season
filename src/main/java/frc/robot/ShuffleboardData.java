package frc.robot;

import java.util.Map;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.internal.DriverStationModeThread;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShuffleboardData extends SubsystemBase {

    CommandSwerveDrivetrain m_CommandSwerveDrivetrain;
    Indexer m_Indexer;
    Intake m_Intake;
    Shooter m_Shooter;
    
    ShuffleboardTab driverTab, autoTab, debugTab, shooterTuning;

    ///////////////////////////
    /// Driver Tab Values /////
    ///////////////////////////
    
    /* Driverstation Widgets */
    SimpleWidget isEnabled;
    SimpleWidget isBrownedOut;
    SimpleWidget batteryVoltage;
    
    /*Drivetrain Widgets */
    SimpleWidget drivetrainVelocityX;
    SimpleWidget drivetrainVelocityY;


    ///////////////////////////
    /// Debug Tab Values //////
    ///////////////////////////
    
    /* Drivetrain Widgets */

    /* Intake Widgets */
    SimpleWidget intakeAtStow;
    SimpleWidget intakeAtDeploy;
    SimpleWidget intakeExtendPosition;
    SimpleWidget intakeRunning;
    SimpleWidget intakeExtenderSpeed;
    SimpleWidget intakeRollerSpeed;

    /* Indexer */
    SimpleWidget stageOneSpeed;
    SimpleWidget stageTwoSpeed;

    ///////////////////////////
    /// Debug Tab Values //////
    ///////////////////////////

    /* Shooter Widgets */
    SimpleWidget shooterRPM;


    public ShuffleboardData(CommandSwerveDrivetrain m_CommandSwerveDrivetrain, Intake m_Intake, Indexer m_Indexer ,Shooter m_Shooter){
        
        this.m_CommandSwerveDrivetrain = m_CommandSwerveDrivetrain;
        this.m_Indexer = m_Indexer;
        this.m_Intake = m_Intake;
        this.m_Shooter = m_Shooter;

        driverTab = Shuffleboard.getTab("Driver");
        autoTab = Shuffleboard.getTab("Autonomous");
        debugTab = Shuffleboard.getTab("Debug");
        shooterTuning = Shuffleboard.getTab("Shooter Tuning");

        ///////////////////////////
        /// Driver Tab Values /////
        ///////////////////////////
        
        
        
        /* Driverstation */
        isEnabled = driverTab.add("Enabled", DriverStation.isEnabled())
            .withWidget(BuiltInWidgets.kBooleanBox);
        isBrownedOut = driverTab.add("Browned Out", RobotController.isBrownedOut())
            .withWidget(BuiltInWidgets.kBooleanBox);
        batteryVoltage = driverTab.add("Battery Voltage", RobotController.getBatteryVoltage());
        
        /* Drivetrain */
        drivetrainVelocityX = driverTab.add("Drivetrain Velocity X", m_CommandSwerveDrivetrain.getChassisSpeeds().vxMetersPerSecond);
        drivetrainVelocityY = driverTab.add("Drivetrian Velocity Y", m_CommandSwerveDrivetrain.getChassisSpeeds().vyMetersPerSecond);

        
        ///////////////////////////
        /// Debug Tab Values //////
        ///////////////////////////

        /* Intake */
        intakeAtStow = debugTab.add("Intake At Stow", m_Intake.isStowed())
            .withWidget(BuiltInWidgets.kBooleanBox);
        intakeAtDeploy = debugTab.add("Intake At Deploy", m_Intake.isDeployed())
            .withWidget(BuiltInWidgets.kBooleanBox);
        intakeExtendPosition = debugTab.add("Intake Extension Position", m_Intake.getExtenderPosition());
        intakeRunning = debugTab.add("Intake Runs Command", m_Intake.getCurrentCommand() != null)
            .withWidget(BuiltInWidgets.kBooleanBox);
        intakeExtenderSpeed = debugTab.add("Intake Extender Speed", m_Intake.getVelocity()[0]);
        intakeRollerSpeed = debugTab.add("Intake Roller Speed", m_Intake.getRollerVelocity());
        
        /* Indexer */
        stageOneSpeed = debugTab.add("Indexer Floor Speed", m_Indexer.getStageOneSpeed());
        stageTwoSpeed = debugTab.add("Indexer Roller Speed", m_Indexer.getStageTwoSpeed());
        

        ///////////////////////////
        /// Shooter Tab Values ////
        ///////////////////////////


        shooterRPM = shooterTuning.add("Shooter RPM", m_Shooter.getFlywheelSpeed());

        ///////////////////////
        /// Autonomous Tab ////
        ///////////////////////

    }

    @Override
    public void periodic(){

        ///////////////////
        /// Driver Tab ////
        ///////////////////
        
        isEnabled.getEntry().setBoolean(DriverStation.isEnabled());
        isBrownedOut.getEntry().setBoolean(RobotController.isBrownedOut());
        batteryVoltage.getEntry().setDouble(RobotController.getBatteryVoltage());
        
        drivetrainVelocityX.getEntry().setDouble(m_CommandSwerveDrivetrain.getChassisSpeeds().vxMetersPerSecond);
        drivetrainVelocityY.getEntry().setDouble(m_CommandSwerveDrivetrain.getChassisSpeeds().vyMetersPerSecond);

        ///////////////////////////
        /// Debug Tab Values //////
        ///////////////////////////

        /* Intake */
        intakeAtStow.getEntry().setBoolean(m_Intake.isStowed());
        intakeAtDeploy.getEntry().setBoolean(m_Intake.isDeployed());
        intakeExtendPosition.getEntry().setDouble(m_Intake.getExtenderPosition());
        intakeRunning.getEntry().setBoolean(m_Intake.getCurrentCommand() != null);
        intakeExtenderSpeed.getEntry().setDouble(m_Intake.getVelocity()[0]);
        intakeRollerSpeed.getEntry().setDouble(m_Intake.getRollerVelocity());

        /* Indexer */
        stageOneSpeed.getEntry().setDouble(m_Indexer.getStageOneSpeed());
        stageTwoSpeed.getEntry().setDouble(m_Indexer.getStageTwoSpeed());

        ///////////////////////////
        /// Shooter Tab Values ////
        ///////////////////////////

        shooterRPM.getEntry().setDouble(m_Shooter.getFlywheelSpeed());

    }


}
