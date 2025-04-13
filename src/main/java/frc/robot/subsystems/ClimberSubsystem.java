// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import java.util.function.DoubleSupplier;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import monologue.Annotations.Log;

public class ClimberSubsystem extends SubsystemBase {

  public SparkMax climberMotor;
  public SparkClosedLoopController climberController;
  private SparkMaxConfig climberConfig;
  public boolean atUpperLimit;
  public boolean atLowerLimit;
  public double gearReduction = 20.;

   
  public final Angle minAngle = Degrees.of(-10);
  public final Angle maxAngle = Degrees.of(20);  

  public final double climberKp = .00002; // P gains caused oscilliation
  public final double climberKi = 0.0;
  public final double climberKd = 0.00;
  public final double climberKFF = .95 / 11000;

  public Servo climber_servo;

  public final double servo_lock_position = 1;
  public final double servo_unlock_position = 0.7;
  public boolean servoLocked = false;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    climberMotor = new SparkMax(Constants.CANIDConstants.climberID, MotorType.kBrushless);
    climberController = climberMotor.getClosedLoopController();
    climberConfig = new SparkMaxConfig();

    climberConfig
        .inverted(false)
        .smartCurrentLimit(40, 40)
        .idleMode(IdleMode.kBrake);

        climberConfig.softLimit.forwardSoftLimit(maxAngle.in(Degrees))
        .reverseSoftLimit(minAngle.in(Degrees))
        .forwardSoftLimitEnabled(false)
        .reverseSoftLimitEnabled(false);


    climberConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    climberConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .velocityFF(climberKFF)
        .pid(climberKp, climberKi, climberKd);

    climberConfig.limitSwitch.forwardLimitSwitchEnabled(false);

    climberConfig.signals.primaryEncoderPositionPeriodMs(20);

    climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    climber_servo = new Servo(1);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Climber/Amps", getAmps());
    SmartDashboard.putNumber("Climber/Velocity", getRPM());

  }

  @Log(key = "angle")
  public Angle getAngle() {
    double rawAngle = climberMotor.getEncoder().getPosition();
    return Degrees.of(rawAngle);
  }

  public Command runClimberMotorAtVelocityCommand(double rpm) {
    return Commands.run(() -> climberController.setReference(rpm, ControlType.kVelocity));
  }

  public double getAmps() {
    return climberMotor.getOutputCurrent();
  }

  public void setServo(double servo_pos) {
    climber_servo.set(servo_pos);
  }

  public void lockClimber() {
    climber_servo.set(servo_lock_position);
    servoLocked = true;
  }

  public void unlockClimber() {
    climber_servo.set(servo_unlock_position);
    servoLocked = false;
  }

  public double getVolts() {
    return climberMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
  }

  public double getRPM() {
    return climberMotor.getEncoder().getVelocity();
  }

  public void stop() {
    climberMotor.setVoltage(0.0);
    
  }

  public Command jogClimberCommand(DoubleSupplier speed) {
    return Commands.run(() -> climberMotor.setVoltage(speed.getAsDouble() * RobotController.getBatteryVoltage()));
  }

  
}
