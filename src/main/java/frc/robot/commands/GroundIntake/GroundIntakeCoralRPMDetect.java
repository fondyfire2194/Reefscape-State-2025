// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GroundIntake;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GroundIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

public class GroundIntakeCoralRPMDetect extends Command {
  /** Creates a new IntakeCoralToswitch. */
  private final GroundIntakeSubsystem m_groundintake;

  private MedianFilter sampleFilter;
  private MedianFilter detectFilter;
  private int algaeDetectLevel = 20;
  private int sampleFilterLevel = 5;
  private int sampleCount;
  private final int numberSamplesWanted = 25;// 1 second
  private int detectCount;
  private final int numberDetectsWanted = 25;// 1 second
  private double filteredRPM;
  private Timer noCoralTimer;
  private double groundIntakeMotorSpeed = .85;

  private double sampledRPM;

  private double coralDetectLevel = 0.25;

  public GroundIntakeCoralRPMDetect(GroundIntakeSubsystem groundIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_groundintake = groundIntake;
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
    sampledRPM = 0;
    filteredRPM = 0;
    sampleFilter.reset();
    detectFilter.reset();
    m_groundintake.simCoralAtGroundIntake = false;

    SmartDashboard.putNumber("GrndIn/TEST", 911);
    m_groundintake.groundIntakeRollerMotor.set(groundIntakeMotorSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sampleCount++;

    SmartDashboard.putNumber("GrndIn/sc", sampleCount);
    if (sampleCount <= numberSamplesWanted) {
      sampledRPM = sampleFilter.calculate(m_groundintake.getRollerRPM());
    } else {
      filteredRPM = detectFilter.calculate(m_groundintake.getRollerRPM());
      detectCount++;
    }

    m_groundintake.coralAtGroundIntake = RobotBase.isReal() && detectCount > numberDetectsWanted
        && filteredRPM < sampledRPM * coralDetectLevel;

    SmartDashboard.putNumber("GrndIn/FilteredRPM", filteredRPM);
    SmartDashboard.putNumber("GrndIn/SampledRPM", sampledRPM);
    SmartDashboard.putBoolean("GrndIn/Detected", m_groundintake.coralAtGroundIntake);

    SmartDashboard.putNumber("GrndIn/Timer", noCoralTimer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_groundintake.simCoralAtGroundIntake = RobotBase.isSimulation();
    SmartDashboard.putBoolean("GrndIn/detectedsim", m_groundintake.simCoralAtGroundIntake);
   
    m_groundintake.groundIntakeRollerMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_groundintake.coralAtGroundIntake ||
        noCoralTimer.hasElapsed(m_groundintake.noCoralAtIntakeTime);
  }
}
