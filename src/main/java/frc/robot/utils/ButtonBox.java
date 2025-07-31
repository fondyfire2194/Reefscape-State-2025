/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;



import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * Contains functions for use with the Logitech F310 controller.
 *
 * @author art.kalb96@gmail.com (Arthur Kalb)
 * @author articgrayling8x8@gmail.com (Dorian Chan)
 * @author kevinsundar@gmail.com (Kevin Vincent)
 */
public class ButtonBox extends Joystick {

  // Gamepad axis ports
  private static final int AXIS_LEFT_X = 0;
  private static final int AXIS_LEFT_Y = 1;
  private static final int AXIS_LT = 2;
  private static final int AXIS_RT = 3;
  private static final int AXIS_RIGHT_X = 4;
  private static final int AXIS_RIGHT_Y = 5;
  private static final int AXIS_DPAD = 6;

  // Gamepad buttons
  private static final int BUTTON_A = 1;
  private static final int BUTTON_B = 2;
  private static final int BUTTON_X = 3;
  private static final int BUTTON_Y = 4;
  private static final int BUTTON_L1 = 5;
  private static final int BUTTON_R1 = 6;
  private static final int BUTTON_SHARE = 7;
  private static final int BUTTON_OPTS = 8;
  private static final int BUTTON_L3 = 9;
  private static final int BUTTON_R3 = 10;
  private static final int BUTTON_LEFT_STICK = 11;
  private static final int BUTTON_RIGHT_STICK = 12;
  private static final int BUTTON_MODE = -1;
  private static final int BUTTON_LOGITECH = -1;

  /**
   * Constructor that creates a Joystick object.
   */
  public ButtonBox(int gamepadPort) {
    super(gamepadPort);
  }

  /**
   * Returns the X position of the left stick.
   */
  public double getLeftX() {
    return getRawAxis(AXIS_LEFT_X);
  }

  /**
   * Returns the X position of the right stick.
   */
  public double getRightX() {
    return getRawAxis(AXIS_RIGHT_X);
  }

  /**
   * Returns the X position of the left stick.
   */
  public double getLT() {
    return getRawAxis(AXIS_LT);
  }


//   public JoystickButton getButtonLT() {
//     return new AxisJoystickButton(this, AXIS_LT);
//   }
//   public JoystickButton getButtonRT() {
//     return new AxisJoystickButton(this, AXIS_RT);
//   }

  /**
   * Returns the X position of the right stick.
   */
  public double getRT() {
    return getRawAxis(AXIS_RT);
  }

  /**
   * Returns the Y position of the left stick.
   */
  public double getLeftY() {
    return getRawAxis(AXIS_LEFT_Y);
  }

  /**
   * Returns the Y position of the right stick.
   */
  public double getRightY() {
    return getRawAxis(AXIS_RIGHT_Y);
  }

  /**
   * Checks whether Button A is being pressed and returns true if it is.
   */
  public boolean getButtonStateA() {
    return getRawButton(BUTTON_A);
  }

  /**
   * Checks whether Button B is being pressed and returns true if it is.
   */
  public boolean getButtonStateB() {
    return getRawButton(BUTTON_B);
  }

  /**
   * Checks whether Button X is being pressed and returns true if it is.
   */
  public boolean getButtonStateX() {
    return getRawButton(BUTTON_X);
  }

  /**
   * Checks whether Button Y is being pressed and returns true if it is.
   */
  public boolean getButtonStateY() {
    return getRawButton(BUTTON_Y);
  }

  /**
   * Returns an object of Button A.
   */
  public JoystickButton getButtonA() {
    return new JoystickButton(this, BUTTON_A);
  }

  /**
   * Returns an object of Button B.
   */
  public JoystickButton getButtonB() {
    return new JoystickButton(this, BUTTON_B);
  }

  /**
   * Returns an object of Button X.
   */
  public JoystickButton getButtonX() {
    return new JoystickButton(this, BUTTON_X);
  }

  /**
   * Returns an object of Button Y.
   */
  public JoystickButton getButtonY() {
    return new JoystickButton(this, BUTTON_Y);
  }

  /**
   * Return the DPad axis positions.
   */
  public double getDPadX() {
    return getRawAxis(AXIS_DPAD);
  }

  /**
   * DPad Left and Right only
   * WPILIB cannot access the vertical axis of the Logitech Game Controller Dpad
   */

  public boolean getDPadLeft() {
    double x = getDPadX();
    return (x < -0.5);
  }

  public boolean getDPadRight() {
    double x = getDPadX();
    return (x > 0.5);
  }

  /**
   * Gets the state of the Start button
   * @return the state of the Start button
   */
  public JoystickButton getStartButton(){
    return new JoystickButton(this, BUTTON_R3);
  }

  public JoystickButton getBackButton() {
    return new JoystickButton(this, BUTTON_L3);
  }

  /**
   * Gets the state of the left shoulder
   * @return the state of the left shoulder
   */
  public JoystickButton getButtonL1() {
    return new JoystickButton(this, BUTTON_L1);
  }

  /**
   * Gets the state of the right shoulder
   * @return the state of the right shoulder
   */
  public JoystickButton getButtonR1() {
    return new JoystickButton(this, BUTTON_R1);
  }

  public JoystickButton getLeftStickClick() {
    return new JoystickButton(this, BUTTON_LEFT_STICK);
  }

  public JoystickButton getRightStickClick() {
    return new JoystickButton(this, BUTTON_RIGHT_STICK);
  }

  public JoystickButton getLeftTriggerClick() {
    return new JoystickButton(this, BUTTON_SHARE);
  }

  public JoystickButton getRightTriggerClick() {
    return new JoystickButton(this, BUTTON_OPTS);
  }
}