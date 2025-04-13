// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;

public class JogArm extends Command {
  /** Creates a new JogArm. */
  private ArmSubsystem m_arm;
  private CommandXboxController m_controller;

  public JogArm(ArmSubsystem arm, CommandXboxController controller) {
    m_arm = arm;
    m_controller = controller;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   

    double yval = -m_controller.getLeftY() / 4;

   
    if (yval > 0 && !m_arm.atUpperLimit || yval < 0 && !m_arm.atLowerLimit) {

      double appliedVolts = yval * RobotController.getBatteryVoltage();

      m_arm.armMotor.setVoltage(appliedVolts);

    } else {
      m_arm.armMotor.setVoltage(0);
    }

 
    m_arm.setGoalRadians(m_arm.getAngle().in(Radians));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
