// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Gamepieces;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GamepieceSubsystem;
import frc.robot.subsystems.PreIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

public class IntakeCoralToSwitch extends Command {
  /** Creates a new IntakeCoralToswitch. */
  private final GamepieceSubsystem m_gamepiece;
  private final PreIntakeSubsystem m_preIn;
  private final boolean m_autoUnstick;
  private Timer coralStuckTimer;

  private Timer noCoralTimer;
  private double stuckTimelimit = 1;
  private double coralIntakeSpeed = .55;
  private double gamepieceMotorIntakeSpeed = .45;

  private double coralUnstickSpeed = .05;
  private double coralReverseSpeedLimit = .02;
  private boolean reversing;
  private boolean coralSeenAtPreswitch;
  private boolean unsticking;
  private int simCtr;
  private int revctr;

  public IntakeCoralToSwitch(GamepieceSubsystem gamepiece, PreIntakeSubsystem prein,
      boolean autoUnstick) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_gamepiece = gamepiece;
    m_preIn = prein;
    m_autoUnstick = autoUnstick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gamepiece.simcoralatswitch = false;
    m_gamepiece.simcoralatswitch = false;
    simCtr = 0;
    revctr = 0;
    noCoralTimer = new Timer();
    noCoralTimer.reset();
    noCoralTimer.start();
    coralStuckTimer = new Timer();
    coralStuckTimer.reset();
    coralStuckTimer.start();
    m_gamepiece.enableLimitSwitch();
    reversing = false;
    unsticking = false;
    m_gamepiece.gamepieceMotor.set(gamepieceMotorIntakeSpeed); // 0.25
    m_preIn.coralIntakeMotor.set(coralIntakeSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotBase.isSimulation()) {
      simCtr++;
      if (simCtr >= 35)
        m_preIn.simcoralatpreintake = true;

      SmartDashboard.putNumber("PIM/simctr", simCtr);
      if (revctr >= 4)
        m_gamepiece.simcoralatswitch = true;
      m_preIn.simcoralatpreintake = false;
    }

    coralSeenAtPreswitch = m_autoUnstick &&m_preIn.coralAtPreIntake();

    if (coralSeenAtPreswitch && !unsticking) {
      coralStuckTimer.reset();
      coralStuckTimer.start();
      unsticking = true;
    }

    if (m_autoUnstick && unsticking && coralStuckTimer.hasElapsed(stuckTimelimit) && !reversing) {
      m_preIn.coralIntakeMotor.set(-coralUnstickSpeed);
      m_gamepiece.gamepieceMotor.set(-coralIntakeSpeed);
      coralStuckTimer.reset();
      coralStuckTimer.start();
      reversing = true;
      revctr++;
    }

    if (m_autoUnstick && reversing && (m_preIn.getIntakeRPM() < -coralReverseSpeedLimit ||
        coralStuckTimer.hasElapsed(stuckTimelimit))) {
      m_preIn.coralIntakeMotor.set(coralIntakeSpeed);
      m_gamepiece.gamepieceMotor.set(coralIntakeSpeed);
      coralStuckTimer.reset();
      coralStuckTimer.start();
      reversing = false;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_preIn.simcoralatpreintake = false;
    m_preIn.stopCoralIntakeMotor();
    m_gamepiece.stopGamepieceMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_gamepiece.coralAtIntake()
        || noCoralTimer.hasElapsed(m_preIn.noCoralAtSwitchTime);

  }
}
