// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

import static edu.wpi.first.units.Units.*;

public class PositionHoldArm extends Command {
  /** Creates a new JogArm. */
  private ArmSubsystem m_arm;

  public PositionHoldArm(ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;

    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_arm.setGoalRadians(m_arm.getAngle().in(Radians));

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.position();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
    m_arm.setGoalRadians(m_arm.getAngle().in(Radians));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}