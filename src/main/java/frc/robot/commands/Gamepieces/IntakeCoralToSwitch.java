// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Gamepieces;

import org.ejml.dense.block.MatrixOps_DDRB;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GamepieceSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

public class IntakeCoralToSwitch extends Command {
  /** Creates a new IntakeCoralToswitch. */
  private final GamepieceSubsystem m_gamepiece;
  private final boolean m_autoUnstick;

  private Timer waitForCoralAtIntakeTimer;
  private double waitForCoralTime = 1;

  private Timer coralUnstickTimer;
  private double unstickReverseTime = .1;

  private Timer noCoralLoadedTimer;// coral never loaded
  private double noCoralLoadedTime = 15;

  private double coralIntakeSpeed = .55;
  private double gamepieceMotorIntakeSpeed = .45;

  private double coralUnstickSpeed = -.25;
  private double gamepieceUnstickSpeed = -.25;

  private boolean coralWasSeenAtPreswitch;

  /**
   * state = 0 waitng for coral at pre switch both motors running forward
   * state= 1 coral at pre switch
   * 
   * 
   * 
   */
  private int state;

  public IntakeCoralToSwitch(GamepieceSubsystem gamepiece,
      boolean autoUnstick) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_gamepiece = gamepiece;

    m_autoUnstick = autoUnstick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gamepiece.simcoralatswitch = false;
    m_gamepiece.simcoralatpreintake = false;
    waitForCoralAtIntakeTimer = new Timer();
    noCoralLoadedTimer = new Timer();
    noCoralLoadedTimer.start();
    coralUnstickTimer = new Timer();
    m_gamepiece.enableLimitSwitch();
    state = 0;
    coralWasSeenAtPreswitch = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("PIM/state", state);
    SmartDashboard.putNumber("PIM/nocoral", noCoralLoadedTimer.get());

    /**
     * state 0 run both motors forward
     * 
     */
    if (state == 0) {
      m_gamepiece.runGamepieceMotor(gamepieceMotorIntakeSpeed); // 0.25
      m_gamepiece.runCoralIntakeMotor(coralIntakeSpeed);
      waitForCoralAtIntakeTimer.reset();
      waitForCoralAtIntakeTimer.start();
      if (m_autoUnstick)
        state = 1;
    }

    /**
     * state 1 look for coral at preswitch
     * keep motors running forward
     */
    if (state == 1 && m_gamepiece.coralAtPreIntake())
      coralWasSeenAtPreswitch = true;

    SmartDashboard.putBoolean("PIM/csapsw", coralWasSeenAtPreswitch);

    /**
     * state 1 both motors forward
     * start timer for reaching second switch
     */

    if (state == 1 && coralWasSeenAtPreswitch) {
      m_gamepiece.runGamepieceMotor(gamepieceMotorIntakeSpeed); // 0.25
      m_gamepiece.runCoralIntakeMotor(coralIntakeSpeed);
      waitForCoralAtIntakeTimer.reset();
      waitForCoralAtIntakeTimer.start();
      state = 2;
    }

    /**
     * state 2
     * coral didn't reach second switch in time
     * try to unstick it
     */

    if (state == 2) {
      m_gamepiece.runGamepieceMotor(gamepieceMotorIntakeSpeed); // 0.25
      m_gamepiece.runCoralIntakeMotor(coralIntakeSpeed);
    }

    if (state == 2 && waitForCoralAtIntakeTimer.hasElapsed(waitForCoralTime)) {
      coralUnstickTimer.reset();
      coralUnstickTimer.start();
      state = 3;
    }

    if (state == 3) {
      m_gamepiece.runGamepieceMotor(gamepieceUnstickSpeed); // 0.25
      m_gamepiece.runCoralIntakeMotor(coralUnstickSpeed);
      coralUnstickTimer.reset();
      coralUnstickTimer.start();
      state = 4;
    }

    /**
     * after a timer elapses
     * run both motors forward again and restart the coral at intake check
     */

    if (state == 4 && (coralUnstickTimer.hasElapsed(unstickReverseTime))) {
      waitForCoralAtIntakeTimer.reset();
      waitForCoralAtIntakeTimer.start();
      state = 1;
    }

    if (RobotBase.isSimulation()) {
      m_gamepiece.simcoralatpreintake = noCoralLoadedTimer.get() > 3;
      m_gamepiece.simcoralatswitch = state == 2 && noCoralLoadedTimer.get() > 10;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_gamepiece.simcoralatpreintake = false;
    m_gamepiece.runGamepieceMotor(0);
    m_gamepiece.runCoralIntakeMotor(0);
    state = 0;
    coralWasSeenAtPreswitch = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_gamepiece.coralAtIntake()
        || noCoralLoadedTimer.hasElapsed(noCoralLoadedTime);

  }
}
