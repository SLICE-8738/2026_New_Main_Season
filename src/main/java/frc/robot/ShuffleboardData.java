package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ShuffleboardData {
    
    ShuffleboardTab driverTab, autoTab, debugTab;

    public ShuffleboardData(CommandSwerveDrivetrain m_CommandSwerveDrivetrain){
        
        driverTab = Shuffleboard.getTab("Driver");
        autoTab = Shuffleboard.getTab("Autonomous");
        debugTab = Shuffleboard.getTab("Debug");
        

        ////////////////////
        /// Driver Tab /////
        ////////////////////

        driverTab.addCamera("Shooter Camera", "limelight-shooter", "http://10.87.38.201").
        withPosition(0, 1).
        withSize(3, 3);

        driverTab.add(m_CommandSwerveDrivetrain.m_Field).
        withWidget(BuiltInWidgets.kField).
        withPosition(5, 5).
        withSize(4, 3);

        ///////////////////////
        /// Autonomous Tab ////
        ///////////////////////

    }


}
