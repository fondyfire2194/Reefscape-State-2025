// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Gamepieces;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Factories.CommandFactory.ArmSetpoints;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GamepieceSubsystem;
import frc.robot.utils.SD;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

public class IntakeCoralToSwitch extends Command {
  /** Creates a new IntakeCoralToswitch. */
  private final GamepieceSubsystem m_gamepiece;
  private final ArmSubsystem m_arm;
  private final boolean m_autoUnstick;
  private Timer coralStuckTimer;

  private Timer noCoralTimer;
  private double stuckTimelimit = 1;
  private double coralIntakeSpeed = .55;
  private double gamepieceMotorIntakeSpeed = .45;

  private double coralUnstickSpeed = .05;
  private double coralReverseSpeedLimit = .02;
  private boolean reversing;

  public IntakeCoralToSwitch(GamepieceSubsystem gamepiece, ArmSubsystem arm, boolean autoUnstick) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_gamepiece = gamepiece;
    m_arm = arm;
    m_autoUnstick = autoUnstick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gamepiece.simcoralatswitch = false;
    noCoralTimer = new Timer();
    noCoralTimer.reset();
    noCoralTimer.start();
    coralStuckTimer = new Timer();
    coralStuckTimer.reset();
    coralStuckTimer.start();
    m_gamepiece.enableLimitSwitch();
    reversing = false;
    m_gamepiece.gamepieceMotor.set(gamepieceMotorIntakeSpeed); // 0.25
    m_gamepiece.coralIntakeMotor.set(coralIntakeSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_autoUnstick && coralStuckTimer.hasElapsed(stuckTimelimit) && !reversing) {
      m_gamepiece.coralIntakeMotor.set(-coralUnstickSpeed);
      coralStuckTimer.reset();
      coralStuckTimer.start();
      reversing = true;
    }

    if (m_autoUnstick && reversing && (m_gamepiece.getIntakeRPM() < -coralReverseSpeedLimit ||
        coralStuckTimer.hasElapsed(stuckTimelimit))) {
      m_gamepiece.coralIntakeMotor.set(coralIntakeSpeed);
      coralStuckTimer.reset();
      coralStuckTimer.start();
      reversing = false;
    }

    SmartDashboard.putBoolean("Gamepiece.reversing", reversing);
    SD.sd2("Gamepiece.revtime", coralStuckTimer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     m_gamepiece.simcoralatswitch = RobotBase.isSimulation();
     m_gamepiece.simcoralatpreintake=false;
    if (m_gamepiece.coralAtIntake())
      m_arm.setGoalDegrees(ArmSetpoints.kTravel);
    m_gamepiece.stopCoralIntakeMotor();
    m_gamepiece.stopGamepieceMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_gamepiece.coralAtIntake()
        || noCoralTimer.hasElapsed(m_gamepiece.noCoralAtSwitchTime);

  }
}
