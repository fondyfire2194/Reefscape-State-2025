// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Gamepieces;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GamepieceSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

public class IntakeCoralToPreSwitch extends Command {
  /** Creates a new IntakeCoralToswitch. */
  private final GamepieceSubsystem m_gamepiece;

  private Timer noCoralLoadedTimer;// coral never loaded
  private double noCoralLoadedTime = 15;

  private double coralIntakeSpeed = .55;
  private double gamepieceMotorIntakeSpeed = .45;

  private int simctr;

  public IntakeCoralToPreSwitch(GamepieceSubsystem gamepiece) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_gamepiece = gamepiece;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    noCoralLoadedTimer = new Timer();
    m_gamepiece.simcoralatswitch = false;
    m_gamepiece.simcoralatpreintake = false;
    simctr = 0;
    m_gamepiece.runGamepieceMotor(gamepieceMotorIntakeSpeed); // 0.25
    m_gamepiece.runCoralIntakeMotor(coralIntakeSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotBase.isSimulation()) {
      simctr++;
      if (simctr >= 20)
        m_gamepiece.simcoralatpreintake = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_gamepiece.coralAtPreIntake() || m_gamepiece.coralAtIntake()
        || noCoralLoadedTimer.hasElapsed(noCoralLoadedTime);

  }
}
