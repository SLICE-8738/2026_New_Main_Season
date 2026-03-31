package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;

public class ShuffleboardData {
    
    ShuffleboardTab driverTab, autoTab, debugTab;

    SimpleWidget intakeAtStow;

    public ShuffleboardData(CommandSwerveDrivetrain m_CommandSwerveDrivetrain, Intake m_Intake){
        
        driverTab = Shuffleboard.getTab("Driver");
        autoTab = Shuffleboard.getTab("Autonomous");
        debugTab = Shuffleboard.getTab("Debug");

        intakeAtStow = driverTab.add("Intake At Stow", m_Intake.isStowed())
            .withWidget(BuiltInWidgets.kBooleanBox);
        

        ////////////////////
        /// Driver Tab /////
        ////////////////////
        

        //driverTab.addCamera("Shooter Camera", "limelight-shooter", "http://10.87.38.201");
        /*
        driverTab.add(m_CommandSwerveDrivetrain.m_Field).
        withWidget(BuiltInWidgets.kField).
        withPosition(5, 5).
        withSize(4, 3);
        */

        ///////////////////////
        /// Autonomous Tab ////
        ///////////////////////

    }


}
