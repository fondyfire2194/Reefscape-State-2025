// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CANIDConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class ElevatorSubsystem extends SubsystemBase implements Logged {

  public final double kElevatorGearing = 62. / 9.;

  public final double kElevatorDrumRadiusInches = 1.757;

  public final double kElevatorDrumRadiusMeters = Units.inchesToMeters(kElevatorDrumRadiusInches);

  public final int sprocketTeeth = 40;

  public final double beltPitch = .005;// 5 MM

  public final double metersPerSprocketRev = sprocketTeeth * beltPitch;

  public final double meterspersecondsprocket = metersPerSprocketRev * (80. / kElevatorGearing);

  public double metersPerMotorRev = (metersPerSprocketRev / kElevatorGearing);//

  public double positionConversionFactor = metersPerMotorRev * 2;// stage multiplier
  public double velocityConversionFactor = positionConversionFactor / 60;

  public double maxVelocityMPS = positionConversionFactor * 5700 / 60;

  public final double elevatorKp = .01;
  public final double elevatorKi = 0;
  public final double elevatorKd = 0;

  /*
   * ( (value that goes up) - (value that goes down) )/ 2 = ks 1.6
   * ( (value that goes up) + (value that goes down) )/2 = kg .8
   */

  // Got these values from sysID
  public final double elevatorKs = 0.27788;// .3; //0.36
  public final double elevatorKg = .75; // 0.56
  public final double elevatorKv = 2.2404;// 12 / maxVelocityMPS;
  public final double elevatorKa = 0.41589;// 0.3;

  public final double kCarriageMass = Units.lbsToKilograms(16); // kg

  public final Distance minElevatorHeight = Inches.of(0);
  public final Distance maxElevatorHeight = Inches.of(65);//

  public final SparkMax leftMotor = new SparkMax(CANIDConstants.leftElevatorID, MotorType.kBrushless);
  public final RelativeEncoder leftEncoder = leftMotor.getEncoder();
  public SparkClosedLoopController leftClosedLoopController = leftMotor.getClosedLoopController();

  public final SparkMax rightMotor = new SparkMax(CANIDConstants.rightElevatorID, MotorType.kBrushless);
  public final RelativeEncoder rightEncoder = rightMotor.getEncoder();

  @Log(key = "alert warning")
  private Alert allWarnings = new Alert("AllWarnings", AlertType.kWarning);
  @Log(key = "alert error")
  private Alert allErrors = new Alert("AllErrors", AlertType.kError);
  @Log(key = "alert sticky fault")
  private Alert allStickyFaults = new Alert("AllStickyFaults", AlertType.kError);
  /*
   * target is 1.5 meters in 1 second
   * average velocity = 1.5 meters per second
   * max velocity = 2 * ave = 3 meters per second
   * accel = max velocity/2*t = 3/2 = 1.5 say 2
   * 
   */

  double TRAJECTORY_VEL = 3;
  double TRAJECTORY_ACCEL = 5;

  public final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
      TRAJECTORY_VEL, TRAJECTORY_ACCEL));

  @Log.NT(key = "goal")
  public TrapezoidProfile.State m_goal = new TrapezoidProfile.State();

  @Log.NT(key = "setpoint")
  public TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State();
  public TrapezoidProfile.State nextSetpoint = new TrapezoidProfile.State();

  private ElevatorFeedforward eff;

  public int posrng;

  public final Trigger atMin = new Trigger(() -> getLeftPositionMeters() <= .1);

  public final Trigger atMax = new Trigger(() -> getLeftPositionMeters() > (maxElevatorHeight.in(Meters) - .2));

  Alert elevatorError = new Alert("Elevator/DifferenceHigh", AlertType.kError);

  @Log.NT(key = "at upper limit")
  public boolean atUpperLimit;

  @Log.NT(key = "at lower limit")
  public boolean atLowerLimit;

  private double targetMeters;

  @Log.NT(key = "target inches")
  private double targetInches;

  @Log.NT(key = "left ff")
  private double leftff;

  // @Log.NT(key = "arm clear")
  // public boolean armClear;

  public boolean showTelemetry = true;

  public double tolerance_inches = 3;

  private final SysIdRoutine routine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(0.2).per(Second), Volts.of(2), null),
      new SysIdRoutine.Mechanism(this::setVoltageSysId, this::logMotors, this));

  /**
   * Subsystem constructor.
   */
  public ElevatorSubsystem() {

    SmartDashboard.putNumber("Elevator/Values/posconv", positionConversionFactor);
    SmartDashboard.putNumber("Elevator/Values/posconvinch",
        Units.metersToInches(positionConversionFactor));
    SmartDashboard.putNumber("Elevator/Values/kv", elevatorKv);
    SmartDashboard.putNumber("Elevator/Values/maxmetrprsec", positionConversionFactor * 5700 / 60);

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();

    leftConfig

        .inverted(true)

        .idleMode(IdleMode.kBrake)

        .smartCurrentLimit(60)

        .closedLoopRampRate(0.25)

            .closedLoop

        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)

        .p(elevatorKp)

        .outputRange(-1, 1)

        // Set PID values for velocity control in slot 1
        .p(0.001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1 / maxVelocityMPS, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    leftConfig.

        encoder.

        positionConversionFactor(positionConversionFactor)

        .velocityConversionFactor(velocityConversionFactor);

    leftConfig.softLimit.forwardSoftLimit(maxElevatorHeight.in(Meters))
        .reverseSoftLimit(minElevatorHeight.in(Meters))
        .forwardSoftLimitEnabled(false)
        .reverseSoftLimitEnabled(false);

    rightConfig

        .idleMode(IdleMode.kBrake)

        .follow(CANIDConstants.leftElevatorID, false)

        .smartCurrentLimit(60)

        .closedLoopRampRate(0.05).closedLoop

        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)

        // Set PID values for position control

        .p(elevatorKp)

        .outputRange(-1, 1);

    rightConfig

        .encoder

        .positionConversionFactor(positionConversionFactor)

        .velocityConversionFactor(velocityConversionFactor);

    leftMotor.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    rightMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    eff = new ElevatorFeedforward(elevatorKs, elevatorKg, elevatorKv, elevatorKa);

    resetPosition(minElevatorHeight.in(Meters));

    // setGoalMeters(minElevatorHeight.in(Meters));
    setGoalMeters(leftEncoder.getPosition());

    SmartDashboard.putNumber("Elevator/Values/reverseSoftLimit", getReverseSoftLimit());
    SmartDashboard.putNumber("Elevator/Values/forwardSoftLimit", getForwardSoftLimit());
    SmartDashboard.putNumber("Elevator/Values/maxvelmps", maxVelocityMPS);
  }

  public void runAtVelocity(double metersPerSecond) {
    leftClosedLoopController.setReference(metersPerSecond, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }

  public void setVoltageSysId(Voltage voltage) {
    leftMotor.setVoltage(voltage);
    rightMotor.setVoltage(voltage);
  }

  public void logMotors(SysIdRoutineLog log) {
    SmartDashboard.putNumber("ElevatorTest/volts", leftMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("ElevatorTest/velocity", getLeftVelocityMetersPerSecond());
    SmartDashboard.putNumber("ElevatorTest/position", getLeftPositionMeters());

    log.motor("elevator-motor-left")
        .voltage(Volts.of(leftMotor.getAppliedOutput() * RobotController.getBatteryVoltage()))
        .linearPosition(Meters.of(getLeftPositionMeters()))
        .linearVelocity(MetersPerSecond.of(getLeftVelocityMetersPerSecond()));
    // log.motor("elevator-motor-right")
    // .voltage(Volts.of(rightMotor.getBusVoltage() *
    // RobotController.getBatteryVoltage()))
    // .linearPosition(Meters.of(getRightPositionMeters()))
    // .linearVelocity(MetersPerSecond.of(getVelocityMetersPerSecond()));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  /**
   * Advance the simulation
   */
  public void simulationPeriodic() {
    // REVLibError e = getLastFault();
    // int n = e.ordinal();
    // REVLibError.fromInt(n);
    // SmartDashboard.putString("D", REVLibError.fromInt(n).toString());

  }

  /**
   * Stop the control loop and motor output.
   */
  public void stop() {
    leftMotor.setVoltage(0.0);
    rightMotor.setVoltage(0.0);
  }

  public double getGoalInches() {
    return Units.metersToInches(m_goal.position);
  }

  public double getGoalMeters() {
    return m_goal.position;
  }

  public void setGoalMeters(double targetMeters) {
    m_goal = new TrapezoidProfile.State(targetMeters, 0);
  }

  public void setGoalInches(double targetInches) {
    this.targetInches = targetInches;
    targetMeters = Units.inchesToMeters(targetInches);
    m_goal.position = targetMeters;

    // currentSetpoint.position = leftEncoder.getPosition();
    // inPositionCtr = 0;
  }

  public Command setGoalInchesCommand(double targetInches) {
    return Commands.runOnce(() -> setGoalInches(targetInches));
  }

  public double getLeftPositionMeters() {
    return leftEncoder.getPosition();
  }

  public double getLeftVelocityMetersPerSecond() {
    return leftEncoder.getVelocity();
  }

  @Log.NT(key = "left position inches")
  public double getLeftPositionInches() {
    return Units.metersToInches(getLeftPositionMeters());
  }

  @Log.NT(key = "left applied output")
  public double getLeftAppliedOutput() {
    return leftMotor.getAppliedOutput();
  }

  public double getRightPositionMeters() {
    return leftEncoder.getPosition();
  }

  @Log.NT(key = "right position inches")
  public double getRightPositionInches() {
    return Units.metersToInches(getRightPositionMeters());
  }

  @Log.NT(key = "left position error inches")
  public double getLeftPositionError() {
    return getGoalInches() - getLeftPositionInches();
  }

  @Log.NT(key = "left right position diffinches")
  public double getLeftRightDiffInches() {
    return getLeftPositionInches() - getRightPositionInches();
  }

  @Log.NT(key = "left motor temperature C")
  public double getLeftMotorTemperatureC() {
    return leftMotor.getMotorTemperature();
  }

  @Log.NT(key = "right motor temperature C")
  public double getRightMotorTemperature() {
    return rightMotor.getMotorTemperature();
  }

  public double getReverseSoftLimit() {
    return leftMotor.configAccessor.softLimit.getReverseSoftLimit();
  }

  public double getForwardSoftLimit() {
    return leftMotor.configAccessor.softLimit.getForwardSoftLimit();
  }

  @Log.NT(key = "reset position")
  public void resetPosition(double val) {
    leftEncoder.setPosition(val);
    rightEncoder.setPosition(val);
  }

  public boolean getActiveFault() {
    return leftMotor.hasActiveFault() || rightMotor.hasActiveFault();
  }

  public REVLibError getLastFault() {
    return rightMotor.getLastError();
  }

  public boolean getStickyFault() {
    return leftMotor.hasStickyFault() || rightMotor.hasStickyFault();
  }

  public boolean getWarnings() {
    return leftMotor.hasActiveWarning() || rightMotor.hasActiveWarning();
  }

  public Command clearStickyFaultsCommand() {
    return Commands.parallel(
        Commands.runOnce(() -> leftMotor.clearFaults()),
        Commands.runOnce(() -> rightMotor.clearFaults()));
  }

  @Override
  public void periodic() {

    if (showTelemetry) {

      SmartDashboard.putNumber("Elevator/targetInches", Units.metersToInches(targetMeters));

      SmartDashboard.putNumber("Elevator/Goal", m_goal.position);
      SmartDashboard.putNumber("Elevator/LeftVolts",
          leftMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

      SmartDashboard.putNumber("Elevator/RightVolts",
          rightMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

      SmartDashboard.putNumber("Elevator/LeftAmps",
          leftMotor.getOutputCurrent());

      SmartDashboard.putNumber("Elevator/RightAmps",
          rightMotor.getOutputCurrent());

      SmartDashboard.putNumber("Elevator/positionleftinches", Units.metersToInches(getLeftPositionMeters()));
      SmartDashboard.putNumber("Elevator/Velleftipsec", Units.metersToInches(leftEncoder.getVelocity()));
      SmartDashboard.putNumber("Elevator/positionrightinches", Units.metersToInches(rightEncoder.getPosition()));
      SmartDashboard.putNumber("Elevator/Velrightipsec", Units.metersToInches(rightEncoder.getVelocity()));

      SmartDashboard.putNumber("Elevator/APPO", leftMotor.getAppliedOutput());
    }
    allWarnings.set(getWarnings());
    allErrors.set(getActiveFault());
    allStickyFaults.set(getStickyFault());

    atUpperLimit = getLeftPositionMeters() >= maxElevatorHeight.in(Meters)
        || rightEncoder.getPosition() >= maxElevatorHeight.in(Meters);

    atLowerLimit = getLeftPositionMeters() <= minElevatorHeight.in(Meters)
        || (rightEncoder.getPosition() <= minElevatorHeight.in(Meters)
            && RobotBase.isReal());

    SmartDashboard.putNumber("Elevator/kvcalc",
        RobotController.getBatteryVoltage() / leftMotor.getEncoder().getVelocity());

    elevatorError.set(Math.abs(getLeftRightDiffInches()) > 2);
    elevatorError.setText(String.valueOf(getLeftRightDiffInches()));
  }

  public boolean atPosition(double toleranceInches) {
    return Math.abs(getLeftPositionError()) < toleranceInches;
  }

}