// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Gamepieces;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GamepieceSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

public class IntakeCoralToPreSwitch extends Command {
  /** Creates a new IntakeCoralToswitch. */
  private final GamepieceSubsystem m_gamepiece;
  private Timer noCoralTimer;
  private double coralIntakeSpeed = .55;
  private double gamepieceMototorSpeed = .45;
  private Debouncer preInSwitchDelay;
  private double debounceTime = .125;

  public IntakeCoralToPreSwitch(GamepieceSubsystem gamepiece) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_gamepiece = gamepiece;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gamepiece.simcoralatpreintake = false;
    preInSwitchDelay = new Debouncer(debounceTime);

    noCoralTimer = new Timer();
    noCoralTimer.reset();
    noCoralTimer.start();

    m_gamepiece.enableLimitSwitch();
    m_gamepiece.gamepieceMotor.set(gamepieceMototorSpeed);
    m_gamepiece.coralIntakeMotor.set(coralIntakeSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_gamepiece.simcoralatpreintake = RobotBase.isSimulation();
    if (m_gamepiece.coralAtIntake()) {
      m_gamepiece.stopCoralIntakeMotor();
      m_gamepiece.stopGamepieceMotor();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return preInSwitchDelay.calculate(m_gamepiece.coralAtPreIntake())
        || m_gamepiece.coralAtIntake()
        || noCoralTimer.hasElapsed(m_gamepiece.noCoralAtSwitchTime);

  }
}
