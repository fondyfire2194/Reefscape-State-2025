// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Factories.CommandFactory.AlgaeRPMSetpoints;
import monologue.Annotations.Log;
import monologue.Logged;

public class AlgaeSubsystem extends SubsystemBase implements Logged {

  public SparkMax algaeRightMotor;
  public SparkClosedLoopController algaeRightController;
  SparkMaxConfig algaeRightConfig;

  public SparkMax algaeLeftMotor;
  public SparkClosedLoopController algaeLeftController;
  SparkMaxConfig algaeLeftConfig;

  @Log(key = "alert warning")
  private Alert allWarnings = new Alert("AllWarnings", AlertType.kWarning);
  @Log(key = "alert error")
  private Alert allErrors = new Alert("AllErrors", AlertType.kError);
  @Log(key = "alert sticky fault")
  private Alert allStickyFaults = new Alert("AllStickyFaults", AlertType.kError);

  @Log(key = "algae target rpm")
  public double targetRPM;

  public final double algaeKp = .00002; // P gains caused oscilliation
  public final double algaeKi = 0.0;
  public final double algaeKd = 0.00;
  public final double algaeKFF = .9 / 11000;

  public double lockAlgaeSet = -.25;

  public double getLockAlgaeSet() {
    return lockAlgaeSet;
  }

  public void setLockAlgaeSet(double val) {
    lockAlgaeSet = val;
  }

  public double lockAlgaeAmps = 4;

  public double getLockAlgaeAmps() {
    return lockAlgaeAmps;
  }

  public void setLockAlgaeAmps(double val) {
    lockAlgaeAmps = val;
    setCurrentLimit((int) val);
  }

  public double algaeDetectLevel = .05;

  public double getAlgaeDetectLevel() {
    return algaeDetectLevel;
  }

  public void setAlgaeDetectLevel(double val) {
    algaeDetectLevel = val;
  }

  public double backupSpeed = .1;

  public double getBackupSpeed() {
    return backupSpeed;
  }

  public void setBackupSpeed(double val) {
    backupSpeed = val;
  }

  private int inOutAlgaeAmps = 20;
  public int inOutalgaeAmps = 40;
  private double algaeDelverSpeed = .6;
  public boolean motorLocked = false;
  public double detectThreshold;

  /** Creates a new algae. */
  public AlgaeSubsystem() {

    algaeLeftMotor = new SparkMax(Constants.CANIDConstants.algaeLeftID, MotorType.kBrushless);
    algaeLeftController = algaeLeftMotor.getClosedLoopController();
    algaeLeftConfig = new SparkMaxConfig();

    algaeRightMotor = new SparkMax(Constants.CANIDConstants.algaeRightID, MotorType.kBrushless);
    algaeRightController = algaeRightMotor.getClosedLoopController();
    algaeRightConfig = new SparkMaxConfig();

    algaeRightConfig
        .inverted(false)
        .smartCurrentLimit(inOutalgaeAmps)
        .idleMode(IdleMode.kBrake);

    algaeRightConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    algaeRightConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .velocityFF(algaeKFF)
        .pid(algaeKp, algaeKi, algaeKd);

    algaeRightConfig.signals.primaryEncoderPositionPeriodMs(20);

    algaeRightMotor.configure(algaeRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    algaeLeftConfig
        .follow(CANIDConstants.algaeRightID, true)
        .smartCurrentLimit(inOutalgaeAmps)
        .idleMode(IdleMode.kBrake);

    algaeLeftConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    algaeLeftConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .velocityFF(algaeKFF)
        .pid(algaeKp, algaeKi, algaeKd);

    algaeLeftConfig.signals.primaryEncoderPositionPeriodMs(20);

    algaeLeftMotor.configure(algaeLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putData("AlgaeInAndHoldTune", this);
  }

  public void setCurrentLimit(int amps) {
    algaeRightConfig.smartCurrentLimit(amps);
    algaeRightMotor.configure(algaeRightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    algaeLeftConfig.smartCurrentLimit(amps);
    algaeLeftMotor.configure(algaeRightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public int getSmartCurrentLimit() {
    return algaeRightMotor.configAccessor.getSmartCurrentLimit();
  }

  public int getSmartCurrentFreeLimit() {
    return algaeRightMotor.configAccessor.getSmartCurrentFreeLimit();
  }

  public void lockMotor() {
    motorLocked = true;
    setCurrentLimit((int) lockAlgaeAmps);
    algaeRightMotor.set(lockAlgaeSet);
    //algaeLeftMotor.set(lockAlgaeSet);
  }

  public void stopalgaeMotor() {
    algaeRightMotor.set(0);
    algaeRightMotor.stopMotor();
    algaeLeftMotor.set(0);
    algaeLeftMotor.stopMotor();

  }

  public Command stopalgaeMotorsCommand() {
    return Commands.parallel(
        Commands.runOnce(() -> motorLocked = false),
        Commands.runOnce(() -> stopalgaeMotor()));
  }

  public Command deliverAlgaeToProcessorCommand() {

    return Commands.sequence(
        Commands.parallel(
            Commands.runOnce(() -> motorLocked = false),
            Commands.runOnce(() -> setCurrentLimit(inOutAlgaeAmps)),
            Commands.runOnce(() -> runalgaeMotorAtVelocity(AlgaeRPMSetpoints.kProcessorDeliver))),
        new WaitCommand(2.5),
        Commands.runOnce(() -> stopalgaeMotor()));

  }

  public Command deliverAlgaeToBargeCommand() {
    return Commands.sequence(Commands.parallel(
        Commands.runOnce(() -> motorLocked = false),
        Commands.runOnce(() -> setCurrentLimit(inOutAlgaeAmps)),
        Commands.runOnce(() -> runalgaeMotorAtVelocity(AlgaeRPMSetpoints.kBargeDeliver))),
        new WaitCommand(2),
        Commands.runOnce(() -> stopalgaeMotor()));

  }

  public void run(double speed) {
    setCurrentLimit(30);
    algaeRightMotor.set(speed);
    //algaeLeftMotor.set(speed);
  }

  public void runalgaeMotorAtVelocity(double rpm) {
    setCurrentLimit(40);
    SmartDashboard.putNumber("algae/tgtrpm", rpm);
    if (RobotBase.isReal())
      algaeRightController.setReference(rpm, ControlType.kVelocity);
    algaeLeftController.setReference(rpm, ControlType.kVelocity);
  }

  public Command setTargetRPM(double rpm) {
    return Commands.runOnce(() -> targetRPM = rpm);
  }

  @Override
  public void periodic() {
    allWarnings.set(getWarnings());
    allErrors.set(getActiveFault());
    allStickyFaults.set(getStickyFault());

    SmartDashboard.putBoolean("algae/MotorLocked", motorLocked);

    SmartDashboard.putNumber("Algae/Adjust/LockSpeed", lockAlgaeSet);
    SmartDashboard.putNumber("Algae/Adjust/LockAmps", lockAlgaeAmps);

    SmartDashboard.putNumber("algae/RightVelocity", algaeRightMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("algae/LeftAmps", algaeRightMotor.getOutputCurrent());
    SmartDashboard.putNumber("algae/GPLimitAmps", getSmartCurrentLimit());

    SmartDashboard.putNumber("algae/LeftVelocity", algaeLeftMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("algae/LeftAmps", algaeLeftMotor.getOutputCurrent());

  }

  @Log(key = "algae rpm")
  public double getRPM() {
    if (RobotBase.isReal())
      return algaeRightMotor.getEncoder().getVelocity();
    else
      return targetRPM;
  }

  @Log(key = "fault")
  public boolean getActiveFault() {
    return algaeRightMotor.hasActiveFault() || algaeLeftMotor.hasActiveFault();
  }

  @Log(key = "sticky fault")
  public boolean getStickyFault() {
    return algaeRightMotor.hasStickyFault() || algaeLeftMotor.hasStickyFault();
  }

  @Log(key = "warning")
  public boolean getWarnings() {
    return algaeRightMotor.hasActiveWarning() || algaeLeftMotor.hasActiveWarning();
  }

  public Command clearStickyFaultsCommand() {
    return Commands.parallel(
        Commands.runOnce(() -> algaeRightMotor.clearFaults()),
        Commands.runOnce(() -> algaeLeftMotor.clearFaults()));
  }

  public double getPosition() {
    return algaeRightMotor.getEncoder().getPosition();
  }

  public double getVelocity() {
    return algaeRightMotor.getEncoder().getVelocity();
  }

  public double getRightAlgaeAmps() {
    return algaeRightMotor.getOutputCurrent();
  }

  public double getLeftAlgaeAmps() {
    return algaeLeftMotor.getOutputCurrent();
  }

  public boolean isStopped() {
    return Math.abs(getVelocity()) < 200;
  }

  public Command jogalgaeIntakeMotorsCommand(DoubleSupplier speed) {
    return Commands.parallel(
        Commands.run(() -> algaeRightMotor.setVoltage(speed.getAsDouble() * RobotController.getBatteryVoltage())));
        //Commands.run(() -> algaeLeftMotor.setVoltage(speed.getAsDouble() * RobotController.getBatteryVoltage())));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("AlgaeInAndHoldTune");
    builder.addDoubleProperty("lockalgaeset", this::getLockAlgaeSet, this::setLockAlgaeSet);
    builder.addDoubleProperty("algaeholdamps", this::getLockAlgaeAmps, this::setLockAlgaeAmps);
    builder.addDoubleProperty("algaedetectlevel", this::getAlgaeDetectLevel, this::setAlgaeDetectLevel);
    builder.addDoubleProperty("algaebackupspeed", this::getBackupSpeed, this::setBackupSpeed);

  }

}
