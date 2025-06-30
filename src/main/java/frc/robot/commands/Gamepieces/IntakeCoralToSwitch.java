// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Gamepieces;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GamepieceSubsystem;
import frc.robot.utils.SD;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

public class IntakeCoralToSwitch extends Command {
  /** Creates a new IntakeCoralToswitch. */
  private final GamepieceSubsystem m_gamepiece;
  private final boolean m_autoUnstick;

  private Timer waitForCoralAtIntakeTimer;
  private double waitForCoralTime = .5;
  private Timer coralUnstickTimer;
  private double unstickReverseTime = .25;

  private Timer noCoralLoadedTimer;// coral never loaded
  private double noCoralLoadedTime = 15;

  private double coralIntakeSpeed = .55;
  private double gamepieceMotorIntakeSpeed = .45;

  private double coralUnstickSpeed = -.15;
  private double gamepieceUnstickSpeed = -.05;

  private double coralReverseSpeedLimit = -.02;
  private boolean reversing;
  private boolean coralSeenAtPreswitch;
  private boolean unsticking;
  private int simCtr;
  private int simrevctr;
  private boolean waiting;

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
    simCtr = 0;
    simrevctr = 0;
    waitForCoralAtIntakeTimer = new Timer();
    noCoralLoadedTimer = new Timer();
    // noCoralLoadedTimer.reset();
    // noCoralLoadedTimer.start();
    coralUnstickTimer = new Timer();
    m_gamepiece.enableLimitSwitch();
    reversing = false;
    unsticking = false;
    waiting = false;
    m_gamepiece.runGamepieceMotor(gamepieceMotorIntakeSpeed); // 0.25
    m_gamepiece.runCoralIntakeMotor(coralIntakeSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotBase.isSimulation()) {
     
      SD.sd2("PIM/simctr", simCtr);
      SD.sd2("PIM/simrevctr", simrevctr);

      simrevctr++;

      if (simrevctr >= 20) {
        m_gamepiece.simcoralatswitch = true;
        m_gamepiece.simcoralatpreintake = false;
      }
    }
    /**
     * look for coral at preswitch and time it to intake switch
     * 
     */
    coralSeenAtPreswitch = m_autoUnstick && m_gamepiece.coralAtPreIntake();

    SmartDashboard.putBoolean("PIM/csapsw", coralSeenAtPreswitch);

    if (coralSeenAtPreswitch && !m_gamepiece.coralAtIntake() && !waiting) {
      waitForCoralAtIntakeTimer.reset();
      waitForCoralAtIntakeTimer.start();
      waiting = true;
    }
    SmartDashboard.putBoolean("PIM/waiting", waiting);

    /**
     * coral was seen at preswitch but didn't reach intake switch in preset time
     * Start unstick sequence
     */
    if (waiting && waitForCoralAtIntakeTimer.hasElapsed(waitForCoralTime) && !unsticking) {
      coralUnstickTimer.reset();
      coralUnstickTimer.start();
      unsticking = true;
    }

    SmartDashboard.putBoolean("PIM/unsticking", unsticking);

    /**
     * reverse pre intake and gamepiece motors for a short time
     * or pre intake motor reaches a reverse rpm limit
     * 
     */
    if (unsticking && !reversing) {
      m_gamepiece.runCoralIntakeMotor(coralUnstickSpeed);
      m_gamepiece.runGamepieceMotor(gamepieceUnstickSpeed);
      reversing = true;
    
    }

    /**
     * after the pre intake motor reaches a reverse speed or a timer elapses
     * run both motors forward again and restart the coral at intake check
     */

    if (reversing && (m_gamepiece.getIntakeRPM() < coralReverseSpeedLimit ||
        coralUnstickTimer.hasElapsed(unstickReverseTime))) {
      m_gamepiece.runCoralIntakeMotor(coralIntakeSpeed);
      m_gamepiece.runGamepieceMotor(gamepieceMotorIntakeSpeed);
      reversing = false;
      unsticking = false;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_gamepiece.simcoralatpreintake = false;
    m_gamepiece.stopCoralIntakeMotor();
    m_gamepiece.stopGamepieceMotor();
    reversing = false;
    unsticking = false;
    waiting = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_gamepiece.coralAtIntake()
        || noCoralLoadedTimer.hasElapsed(noCoralLoadedTime);

  }
}
