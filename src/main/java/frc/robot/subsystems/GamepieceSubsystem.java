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

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANIDConstants;
import frc.robot.utils.SD;
import monologue.Annotations.Log;
import monologue.Logged;

public class GamepieceSubsystem extends SubsystemBase implements Logged {

  public SparkMax gamepieceMotor;
  public SparkClosedLoopController gamepieceController;
  SparkMaxConfig gamepieceConfig;

  public SparkLimitSwitch coralDetectSwitch;

  public boolean simcoralatswitch;

  public final double coralIntakeKp = .002; // P gains caused oscilliation
  public final double coralIntakeKi = 0.0;
  public final double coralIntakeKd = 0.00;
  public final double coralIntakeKFF = .8 / 5700;

  public SparkMax coralIntakeMotor;
  public SparkClosedLoopController coralIntakeController;
  SparkMaxConfig coralIntakeConfig;

  public SparkLimitSwitch coralPreDetectSwitch;
  public boolean simcoralatpreintake;

  @Log(key = "alert warning")
  private Alert allWarnings = new Alert("AllWarnings", AlertType.kWarning);
  @Log(key = "alert error")
  private Alert allErrors = new Alert("AllErrors", AlertType.kError);
  @Log(key = "alert sticky fault")
  private Alert allStickyFaults = new Alert("AllStickyFaults", AlertType.kError);

  @Log(key = "target rpm")
  public double targetRPM;

  public final double gamepieceKp = .00002; // P gains caused oscilliation
  public final double gamepieceKi = 0.0;
  public final double gamepieceKd = 0.00;
  public final double gamepieceKFF = .9 / 11000;

  public double coralDeliverSpeed = .7;
  public double coralFastDeliverSpeed = .8;
  public double coralL1DeliverSpeed = .5;

  public int inOutCoralAmps = 40;
  public boolean showTelemetry;

  /** Creates a new gamepiece. */
  public GamepieceSubsystem() {

    gamepieceMotor = new SparkMax(Constants.CANIDConstants.gamepieceID, MotorType.kBrushless);
    gamepieceController = gamepieceMotor.getClosedLoopController();
    coralDetectSwitch = gamepieceMotor.getForwardLimitSwitch();
    gamepieceConfig = new SparkMaxConfig();

    gamepieceConfig
        .inverted(true)
        .smartCurrentLimit(inOutCoralAmps)
        .idleMode(IdleMode.kBrake);

    gamepieceConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    gamepieceConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .velocityFF(gamepieceKFF)
        .pid(gamepieceKp, gamepieceKi, gamepieceKd);

    gamepieceConfig.limitSwitch.forwardLimitSwitchEnabled(false);
    gamepieceConfig.limitSwitch.reverseLimitSwitchEnabled(false);


    gamepieceConfig.signals.primaryEncoderPositionPeriodMs(20);

    gamepieceMotor.configure(gamepieceConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    disableLimitSwitch();

    coralIntakeMotor = new SparkMax(CANIDConstants.coralIntakeID, MotorType.kBrushless);
    coralIntakeController = coralIntakeMotor.getClosedLoopController();
    coralIntakeConfig = new SparkMaxConfig();

    coralIntakeConfig
        .inverted(true)
        .smartCurrentLimit(20, 20)
        .idleMode(IdleMode.kBrake);

    coralIntakeConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    coralIntakeConfig.softLimit
        .forwardSoftLimitEnabled(false)
        .reverseSoftLimitEnabled(false);

    coralIntakeConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .velocityFF(coralIntakeKFF)
        .pid(coralIntakeKp, coralIntakeKi, coralIntakeKd);

    coralIntakeConfig.signals.primaryEncoderPositionPeriodMs(10);

    coralIntakeMotor.configure(coralIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    coralPreDetectSwitch = coralIntakeMotor.getForwardLimitSwitch();

    
  }

  public void runCoralIntakeMotor(double speed){
    SD.sd2("Gamepiece/cimtgtspd", speed);
    coralIntakeMotor.set(speed);
  }

  public void runCoralIntakeMotorAtVelocity(double rpm) {
    if (RobotBase.isReal())
      coralIntakeController.setReference(rpm, ControlType.kVelocity);
  }

  public void stopCoralIntakeMotor() {
    coralIntakeMotor.set(0);
    coralIntakeMotor.stopMotor();
  }

  public void stopGamepieceMotor() {
    gamepieceMotor.set(0);
    gamepieceMotor.stopMotor();

  }

  public Command stopGamepieceMotorsCommand() {
    return Commands.parallel(
        Commands.runOnce(() -> stopGamepieceMotor()));
  }

  public Command reverseOffSwitch() {
    return Commands.sequence(
        Commands.runOnce(() -> disableLimitSwitch()),
        Commands.runOnce(() -> gamepieceMotor.set(-.1)),
        Commands.waitUntil(() -> !coralAtIntake()),
        stopGamepieceMotorsCommand());
  }

  public void runGamepieceMotor(double speed) {
    SD.sd2("Gamepiece/gptgtspd", speed);
    gamepieceMotor.set(speed);
  }

  public void runGamepieceMotorAtVelocity(double rpm) {
    SD.sd2("Gamepiece/tgtrpm", rpm);
    if (RobotBase.isReal())
      gamepieceController.setReference(rpm, ControlType.kVelocity);
  }

  public Command setTargetRPM(double rpm) {
    return Commands.runOnce(() -> targetRPM = rpm);
  }

  @Log(key = "coral at intake")
  public boolean coralAtIntake() {
    return RobotBase.isReal() && coralDetectSwitch.isPressed() ||
        RobotBase.isSimulation() && simcoralatswitch;
  }

  @Override
  public void periodic() {
    allWarnings.set(getWarnings());
    allErrors.set(getActiveFault());
    allStickyFaults.set(getStickyFault());

    SmartDashboard.putBoolean("Gamepiece/CoralAtIntake", coralAtIntake());
    if (showTelemetry) {
      SD.sd2("Gamepiece/GPVelocity", gamepieceMotor.getEncoder().getVelocity());
      SD.sd2("Gamepiece/GPAmps", gamepieceMotor.getOutputCurrent());
      SmartDashboard.putBoolean("PreIn/CoralAtPreIntake", coralAtPreIntake());
      SD.sd2("Gamepiece/Velocity", coralIntakeMotor.getEncoder().getVelocity());
      SD.sd2("Gamepiece/Amps", coralIntakeMotor.getOutputCurrent());

    }
  }

  public void enableLimitSwitch() {
    gamepieceConfig.limitSwitch.forwardLimitSwitchEnabled(true);
    gamepieceMotor.configure(gamepieceConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void disableLimitSwitch() {
    gamepieceConfig.limitSwitch.forwardLimitSwitchEnabled(false);
    gamepieceMotor.configure(gamepieceConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Log(key = "gpswitchenabled")
  public boolean getLimitSwitchEnabled() {
    return gamepieceMotor.configAccessor.limitSwitch.getForwardLimitSwitchEnabled();
  }

  @Log(key = "gamepiece amps")
  public double getAmps() {
    return gamepieceMotor.getOutputCurrent();
  }

  @Log(key = "gamepiece rpm")
  public double getGamepieceRPM() {
    if (RobotBase.isReal())
      return gamepieceMotor.getEncoder().getVelocity();
    else
      return targetRPM;
  }

  public Command jogCoralIntakeMotorCommand(DoubleSupplier speed) {
    return Commands
        .run(() -> coralIntakeMotor.setVoltage(speed.getAsDouble() * RobotController.getBatteryVoltage()));
  }

  public Command stopIntakeMotorCommand() {
    return Commands.runOnce(() -> coralIntakeMotor.stopMotor());
  }

  @Log(key = "coralatpreintake")
  public boolean coralAtPreIntake() {
    return RobotBase.isReal() && coralPreDetectSwitch.isPressed() ||
        RobotBase.isSimulation() && simcoralatpreintake;
  }

  public double getIntakeAmps() {
    return coralIntakeMotor.getOutputCurrent();
  }

  public double getIntakeRPM() {
    return coralIntakeMotor.getEncoder().getVelocity();
  }

  @Log(key = "fault")
  public boolean getActiveFault() {
    return gamepieceMotor.hasActiveFault();
  }

  @Log(key = "sticky fault")
  public boolean getStickyFault() {
    return gamepieceMotor.hasStickyFault();
  }

  @Log(key = "warning")
  public boolean getWarnings() {
    return gamepieceMotor.hasActiveWarning();
  }

  public Command clearStickyFaultsCommand() {
    return Commands.runOnce(() -> gamepieceMotor.clearFaults());
  }

  public double getPosition() {
    return gamepieceMotor.getEncoder().getPosition();
  }

  public double getVelocity() {
    return gamepieceMotor.getEncoder().getVelocity();
  }

  public double getGamepieceAmps() {
    return gamepieceMotor.getOutputCurrent();
  }

  public boolean isStopped() {
    return Math.abs(getVelocity()) < 200;
  }

  public Command jogGamepieceMotorCommand(DoubleSupplier speed) {
    return Commands.sequence(Commands.runOnce(() -> disableLimitSwitch()),
        Commands.run(() -> gamepieceMotor.setVoltage(speed.getAsDouble() * RobotController.getBatteryVoltage())));
  }

}
