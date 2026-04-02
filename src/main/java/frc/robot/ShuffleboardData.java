package frc.robot;

import java.util.Map;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.internal.DriverStationModeThread;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShuffleboardData extends SubsystemBase {

    Intake m_Intake;
    Shooter m_Shooter;
    
    ShuffleboardTab driverTab, autoTab, debugTab, shooterTuning;

    ///////////////////////////
    /// Driver Tab Values //////
    ///////////////////////////

    //SimpleWidget batteryVoltage;

    ///////////////////////////
    /// Debug Tab Values //////
    ///////////////////////////

    /* Intake Widgets */
    SimpleWidget intakeAtStow;
    SimpleWidget intakeAtDeploy;
    SimpleWidget intakeExtendPosition;
    SimpleWidget intakeRunning;

    /* Shooter Widgets */
    SimpleWidget shooterRPM;


    public ShuffleboardData(CommandSwerveDrivetrain m_CommandSwerveDrivetrain, Intake m_Intake, Shooter m_Shooter){
        
        this.m_Intake = m_Intake;
        this.m_Shooter = m_Shooter;

        driverTab = Shuffleboard.getTab("Driver");
        autoTab = Shuffleboard.getTab("Autonomous");
        debugTab = Shuffleboard.getTab("Debug");
        shooterTuning = Shuffleboard.getTab("Shooter Tuning");

        ///////////////////////////
        /// Driver Tab Values /////
        ///////////////////////////
        
        //batteryVoltage = driverTab.add("Battery Voltage", );

        ///////////////////////////
        /// Debug Tab Values //////
        ///////////////////////////

        intakeAtStow = debugTab.add("Intake At Stow", m_Intake.isStowed())
            .withWidget(BuiltInWidgets.kBooleanBox);
        intakeAtDeploy = debugTab.add("Intake At Deploy", m_Intake.isDeployed())
            .withWidget(BuiltInWidgets.kBooleanBox);
        intakeExtendPosition = debugTab.add("Intake Extension Position", m_Intake.getExtenderPosition());
        intakeRunning = debugTab.add("Intake Runs Command", m_Intake.getCurrentCommand() != null)
            .withWidget(BuiltInWidgets.kBooleanBox);
        
        

        

        ///////////////////////////
        /// Shooter Tab Values //////
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

        intakeAtStow.getEntry().setBoolean(m_Intake.isStowed());
        intakeAtDeploy.getEntry().setBoolean(m_Intake.isDeployed());
        intakeExtendPosition.getEntry().setDouble(m_Intake.getExtenderPosition());
        intakeRunning.getEntry().setBoolean(m_Intake.getCurrentCommand() != null);


        shooterRPM.getEntry().setDouble(m_Shooter.getFlywheelSpeed());

    }


}
