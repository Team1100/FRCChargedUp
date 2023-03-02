/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.Arm.ArmToPositionXY;
import frc.robot.commands.Hand.ExpelCube;
import frc.robot.commands.Hand.IntakeCube;
import frc.robot.commands.Hand.SpinIntake;
import frc.robot.input.AttackThree;
import frc.robot.input.ButtonBox;
import frc.robot.input.XboxController;
import frc.robot.input.KeyboardBox;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  private static OI oi;

  public static AttackThree leftStick;
  public static AttackThree rightStick;
  private static XboxController DriverXboxController;
  private static XboxController OperatorXboxController;
  private static ButtonBox buttonBox;
  private static KeyboardBox keyboardBox;


  /**
   * Used outside of the OI class to return an instance of the class.
   * @return Returns instance of OI class formed from constructor.
   */
  public static OI getInstance() {
    if (oi == null) {
      oi = new OI();
    }
    return oi;
  }

  public OI() {
    // User Input
    // TODO: Tune deadband
    if (Constants.ATTACK_THREE_ENABLE) {
      leftStick = new AttackThree(RobotMap.U_JOYSTICK_LEFT, 0.01);
      rightStick = new AttackThree(RobotMap.U_JOYSTICK_RIGHT, 0.01);
    }
    if (Constants.BUTTON_BOX_ENABLE) {
      buttonBox = new ButtonBox(RobotMap.U_BUTTON_BOX);
    }
    if (Constants.KEYBOARD_BOX_ENABLE) {
      keyboardBox = new KeyboardBox(RobotMap.U_KEYBOARD_BOX);
    }
    if (Constants.XBOX_CONTROLLER_DRIVER_ENABLE) {
      DriverXboxController = new XboxController(RobotMap.U_DRIVER_XBOX_CONTROLLER, Constants.XBOX_DEADBAND_LIMIT);
    }
    if (Constants.XBOX_CONTROLLER_OPERATOR_ENABLE) {
      OperatorXboxController = new XboxController(RobotMap.U_OPERATOR_XBOX_CONTROLLER, Constants.XBOX_DEADBAND_LIMIT);
    }
    
    ////////////////////////////////////////////////////
    // Now Mapping Commands to XBox
    ////////////////////////////////////////////////////
    if (Constants.XBOX_CONTROLLER_DRIVER_ENABLE) {
    }
    if (Constants.XBOX_CONTROLLER_OPERATOR_ENABLE) {
    }
    
    ////////////////////////////////////////////////////
    // Now Mapping Commands to AttackThree controllers
    ////////////////////////////////////////////////////
    if (Constants.ATTACK_THREE_ENABLE) {
    }

    ////////////////////////////////////////////////////
    // Now Mapping Commands to Button Box
    ////////////////////////////////////////////////////
    if (Constants.BUTTON_BOX_ENABLE) {
    }


    ////////////////////////////////////////////////////
    // Now Mapping Commands to Keyboard Box
    ////////////////////////////////////////////////////

    if (Constants.KEYBOARD_BOX_ENABLE) {
    }
    
  }

  /**
   * Returns the left Joystick
   * @return the leftStick
   */
  public AttackThree getLeftStick() {
    return leftStick;
  }

  /**
   * Returns the right Joystick
   * @return the rightStick
   */
  public AttackThree getRightStick() {
    return rightStick;
  }

  /**
   * Returns the Xbox Controller
   * @return the Xbox Controller
   */
  public XboxController getDriverXboxController() {
      return DriverXboxController;
  }

  public XboxController getOperatorXboxController() {
    return OperatorXboxController;
}

  /**
   * Returns the KeyboardBox
   * @return the KeyboardBox
   */
  public KeyboardBox getKeyboardBox() {
    return keyboardBox;
  }
  
}
