// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ClimberSubsystem;

public class JogClimber extends Command {
  /** Creates a new JogCLimber. */
  private ClimberSubsystem m_climber;
  private CommandXboxController m_controller;

  public JogClimber(ClimberSubsystem climber, CommandXboxController controller) {
    m_climber = climber;
    m_controller = controller;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   

    double yval = m_controller.getLeftY() / 4;

   
    if (yval > 0 && !m_climber.atUpperLimit || yval < 0 && !m_climber.atLowerLimit) {

      double appliedVolts = yval * RobotController.getBatteryVoltage();

      m_climber.climberMotor.setVoltage(appliedVolts);

    } else {

      m_climber.climberMotor.setVoltage(0);
    }
   
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
