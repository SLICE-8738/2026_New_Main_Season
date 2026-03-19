// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/* Add your docs here. */
public class Buttons {
    public static XboxController controller1 = new XboxController(Constants.OIConstants.kDriverControllerPort);

    /* DRIVER CONTROLLER */
    public static Trigger controller1_AButton = new JoystickButton(controller1, 1);
    public static Trigger controller1_BButton = new JoystickButton(controller1, 2);
    public static Trigger controller1_XButton = new JoystickButton(controller1, 3);
    public static Trigger controller1_YButton = new JoystickButton(controller1, 4);
    public static Trigger controller1_leftBumper = new JoystickButton(controller1, 5);
    public static Trigger controller1_rightBumper = new JoystickButton(controller1, 6);
    public static Trigger controller1_minusButton =  new JoystickButton(controller1, 7);
    public static Trigger controller1_plusButton = new JoystickButton(controller1, 8);
    public static Trigger controller1_pressLeftStick = new JoystickButton(controller1, 9);
    public static Trigger controller1_pressRightStick = new JoystickButton(controller1, 10);
    public static Trigger controller1_LeftTrigger =  new Trigger(() -> controller1.getRawAxis(2) > 0.1);
    public static Trigger controller1_RightTrigger = new Trigger(() -> controller1.getRawAxis(3) > 0.1);

    /* OPERATOR CONTROLLER */
    /*
    
    Uncomment if operator controller is needed
    
    public static XboxController controller2 = new XboxController(Constants.OIConstants.kOperatorControllerPort);
    
    public static Trigger controller2_AButton = new JoystickButton(controller2, 1);
    public static Trigger controller2_BButton = new JoystickButton(controller2, 2);
    public static Trigger controller2_XButton = new JoystickButton(controller2, 3);
    public static Trigger controller2_YButton = new JoystickButton(controller2, 4);
    public static Trigger controller2_leftBumper = new JoystickButton(controller2, 5);
    public static Trigger controller2_rightBumper = new JoystickButton(controller2, 6);
    public static Trigger controller2_minusButton =  new JoystickButton(controller2, 7);
    public static Trigger controller2_plusButton = new JoystickButton(controller2, 8);
    public static Trigger controller2_pressLeftStick = new JoystickButton(controller2, 9);
    public static Trigger controller2_pressRightStick = new JoystickButton(controller2, 10);
    public static Trigger controller2_RightTrigger =  new Trigger(() -> controller2.getRawAxis(2) > 0.1);
    public static Trigger controller2_LeftTrigger = new Trigger(() -> controller2.getRawAxis(3) > 0.1);
    
    */

}
