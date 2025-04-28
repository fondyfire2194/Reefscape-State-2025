// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Gamepieces;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GamepieceSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

public class IntakeCoralToAmpsDetect extends Command {
  /** Creates a new IntakeCoralToswitch. */
  private final GamepieceSubsystem m_gamepiece;

  private MedianFilter sampleFilter;
  private MedianFilter detectFilter;
  private int algaeDetectLevel = 20;
  private int sampleFilterLevel = 5;
  private int sampleCount;
  private final int numberSamplesWanted = 25;// 1 second
  private int detectCount;
  private final int numberDetectsWanted = 25;// 1 second
  private double filteredAmps;
  private Timer noCoralTimer;
  private double coralIntakeSpeed = .55;
  private double gamepieceMototorSpeed = .45;
  private double debounceTime = .125;

  private double sampledAmps;

  private boolean coralDetected;

  private double coralDetectLevel = 1.5;

  public IntakeCoralToAmpsDetect(GamepieceSubsystem gamepiece) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_gamepiece = gamepiece;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    noCoralTimer = new Timer();
    noCoralTimer.reset();
    noCoralTimer.start();

    sampleFilter = new MedianFilter(sampleFilterLevel);
    detectFilter = new MedianFilter(algaeDetectLevel);
    sampleCount = 0;
    detectCount = 0;
    sampledAmps = 0;
    filteredAmps = 0;
    sampleFilter.reset();
    detectFilter.reset();

    m_gamepiece.enableLimitSwitch();
    m_gamepiece.gamepieceMotor.set(gamepieceMototorSpeed);
    m_gamepiece.coralIntakeMotor.set(coralIntakeSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sampleCount++;
    if (sampleCount <= numberSamplesWanted) {
      sampledAmps = sampleFilter.calculate(m_gamepiece.coralIntakeMotor.getOutputCurrent());
    } else {
      filteredAmps = detectFilter.calculate(m_gamepiece.coralIntakeMotor.getOutputCurrent());
      detectCount++;
    }

    coralDetected = detectCount > numberDetectsWanted && filteredAmps > sampledAmps * coralDetectLevel;

    SmartDashboard.putNumber("Coral/FilteredAmps", filteredAmps);
    SmartDashboard.putNumber("Coral/SampledRPM", sampledAmps);
    SmartDashboard.putBoolean("Coral/Detected", coralDetected);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_gamepiece.coralAtIntake()) {
      m_gamepiece.stopCoralIntakeMotor();
      m_gamepiece.stopGamepieceMotor();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coralDetected
        || m_gamepiece.coralAtIntake()
        || noCoralTimer.hasElapsed(m_gamepiece.noCoralAtSwitchTime);

  }
}
