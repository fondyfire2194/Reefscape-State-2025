package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CANIDConstants;
import frc.robot.utils.SD;
import monologue.Annotations.Log;
import monologue.Logged;

public class PreIntakeSubsystem extends SubsystemBase implements Logged {

    public final SparkMax preIntakeArmMotor = new SparkMax(CANIDConstants.preIntakeMotorID, MotorType.kBrushless);

    private SparkClosedLoopController preIntakeArmClosedLoopController = preIntakeArmMotor.getClosedLoopController();

    @Log(key = "alert warning")
    private Alert allWarnings = new Alert("AllWarnings", AlertType.kWarning);
    @Log(key = "alert error")
    private Alert allErrors = new Alert("AllErrors", AlertType.kError);
    @Log(key = "alert sticky fault")
    private Alert allStickyFaults = new Alert("AllStickyFaults", AlertType.kError);

    SparkMaxConfig preintakeArmConfig;

    public boolean atUpperLimit;

    public boolean atLowerLimit;

    public double angleToleranceDeg = 10;

    public boolean presetOnce;

    public double gearReduction = 125;// 100.;

    double degperencderrev = (360) / gearReduction;

    double posConvFactor = degperencderrev;

    double velConvFactor = posConvFactor / 60;

    double maxmotorrps = 11000 / 60;// 5700 / 60;

    double maxdegpersec = degperencderrev * maxmotorrps;//

    public double preintakeKp = 0.075;// 0.075;

    public final double preintakeKi = 0;
    public final double preintakeKd = 0;

    double TRAJECTORY_VEL = 40;
    double TRAJECTORY_ACCEL = 60;

    public final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            TRAJECTORY_VEL, TRAJECTORY_ACCEL));

    @Log.NT(key = "goal")
    public TrapezoidProfile.State m_goal = new TrapezoidProfile.State();

    @Log.NT(key = "setpoint")
    public TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State();
    public TrapezoidProfile.State nextSetpoint = new TrapezoidProfile.State();

    /**
     * Angles are set so that 90 degrees is with the preintake balanced over
     * center
     * This means kg will act equally on both sides of top center
     * 
     */

    public final double minAngle = 0;
    public final double maxAngle = 100;

    public IdleMode currentMode = IdleMode.kBrake;

    public boolean showTelemetry;

    public PreIntakeSubsystem() {

        preintakeArmConfig = new SparkMaxConfig();

        preintakeArmConfig
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40);

        preintakeArmConfig.encoder
                .positionConversionFactor(posConvFactor)
                .velocityConversionFactor(velConvFactor);

        preintakeArmConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control
                .p(preintakeKp)
                .outputRange(-0.4, 0.4);

        preintakeArmConfig.softLimit.forwardSoftLimit(maxAngle)
                .reverseSoftLimit(minAngle)
                .forwardSoftLimitEnabled(false)
                .reverseSoftLimitEnabled(false);

        preintakeArmConfig.signals.primaryEncoderPositionPeriodMs(20);

        preIntakeArmMotor.configure(preintakeArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        preIntakeArmMotor.getEncoder().setPosition(0);

        m_goal.position = 0;

    }

    public boolean getActiveFault() {
        return preIntakeArmMotor.hasActiveFault();
    }

    public boolean getStickyFault() {
        return preIntakeArmMotor.hasStickyFault();
    }

    public boolean getWarnings() {
        return preIntakeArmMotor.hasActiveWarning();
    }

    public void position() {
        // Send setpoint to spark max controller
        nextSetpoint = m_profile.calculate(.02, currentSetpoint, m_goal);

        preIntakeArmClosedLoopController.setReference(
                nextSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0);

        currentSetpoint = nextSetpoint;
    }

    public Command positionCommand() {
        return Commands.run(() -> position(), this);
    }

    public void setMotorToCoast() {
        currentMode = IdleMode.kCoast;
        preintakeArmConfig.idleMode(IdleMode.kCoast);
        preIntakeArmMotor.configure(preintakeArmConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public Command preIntakeToStartCommand() {
        return Commands.sequence(Commands.runOnce(() -> setMotorToCoast()),
                Commands.runOnce(() -> preIntakeArmMotor.setVoltage(-0.2)), new WaitCommand(1),
                Commands.runOnce(() -> stop()),
                new WaitCommand(1),
                Commands.runOnce(() -> setMotorToBrake()),
                Commands.runOnce(() -> preIntakeArmMotor.getEncoder().setPosition(0)));

    }

    public void setMotorToBrake() {
        currentMode = IdleMode.kBrake;
        preintakeArmConfig.idleMode(IdleMode.kBrake);
        preIntakeArmMotor.configure(preintakeArmConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public Command goHome() {
        return Commands.runOnce(() -> m_goal.position = 0);
    }

    public Command goFullDown() {
        return Commands.runOnce(() -> m_goal.position = 90);
    }

    public Command jogMotorCommand(DoubleSupplier speed) {
        return Commands
                .run(() -> preIntakeArmMotor.setVoltage(speed.getAsDouble() * RobotController.getBatteryVoltage()),
                        this);
    }

    public Command stopArmMotorCommand() {
        return Commands.runOnce(() -> preIntakeArmMotor.stopMotor());
    }

    @Override
    public void periodic() {

        // This method will be called once per scheduler run
        allWarnings.set(getWarnings());
        allErrors.set(getActiveFault());
        allStickyFaults.set(getStickyFault());

        atUpperLimit = getAngle() > maxAngle;
        atLowerLimit = getAngle() < minAngle;
        if (showTelemetry) {
            SD.sd2("PreIn/pos", preIntakeArmMotor.getEncoder().getPosition());
            // SmartDashboard.putBoolean("PreIn/atpos", preintakeAtStartPosition());
            SD.sd2("PreIn/vel", preIntakeArmMotor.getEncoder().getVelocity());
            SD.sd2("PreIn/volts",
                    preIntakeArmMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
            SD.sd2("PreIn/amps", getAmps());
        }

    }

    @Override
    public void simulationPeriodic() {

    }

    public void resetEncoder(double val) {
        preIntakeArmMotor.getEncoder().setPosition(val);
    }

    public void setTolerance(double toleranceDeg) {
        angleToleranceDeg = toleranceDeg;
    }

    public Command setGoalDegreesCommand(double targetDegrees) {
        return Commands.runOnce(() -> m_goal.position = targetDegrees);

    }

    public double getMotorEncoderAngleRadians() {
        return preIntakeArmMotor.getEncoder().getPosition();
    }

    @Log.NT(key = "motor degrees")
    public double getMotorDegrees() {
        return Units.radiansToDegrees(preIntakeArmMotor.getEncoder().getPosition());
    }

    public double getMotorEncoderRadsPerSec() {
        return preIntakeArmMotor.getEncoder().getVelocity();
    }

    public double getMotorEncoderDegsPerSec() {
        return Units.radiansToDegrees(preIntakeArmMotor.getEncoder().getVelocity());
    }

    @Log(key = "angle")
    public double getAngle() {
        return preIntakeArmMotor.getEncoder().getPosition();

    }

    public double getRadsPerSec() {
        return preIntakeArmMotor.getEncoder().getVelocity();
    }

    @Log.NT(key = "preintake degrees per sec")
    public double getDegreesPerSec() {
        return Units.radiansToDegrees(preIntakeArmMotor.getEncoder().getVelocity());
    }

    public boolean onPlusSoftwareLimit() {
        return preIntakeArmMotor.getEncoder().getPosition() >= preIntakeArmMotor.configAccessor.softLimit
                .getForwardSoftLimit();
    }

    public boolean onMinusSoftwareLimit() {
        return preIntakeArmMotor.getEncoder().getPosition() <= preIntakeArmMotor.configAccessor.softLimit
                .getReverseSoftLimit();
    }

    @Log(key = "on limit")
    public boolean onLimit() {
        return onPlusSoftwareLimit() || onMinusSoftwareLimit();
    }

    public void stop() {
        preIntakeArmMotor.setVoltage(0);
    }

    public double getAmps() {
        return preIntakeArmMotor.getOutputCurrent();
    }

    public boolean isBraked() {
        return preIntakeArmMotor.configAccessor.getIdleMode() == IdleMode.kBrake;
    }

    public boolean getSoftwareLimitsEnabled() {
        return preIntakeArmMotor.configAccessor.softLimit.getForwardSoftLimitEnabled()
                || preIntakeArmMotor.configAccessor.softLimit.getReverseSoftLimitEnabled();
    }

    public boolean getStickyFaults() {
        return preIntakeArmMotor.hasStickyFault();
    }

    public Command clearStickyFaultsCommand() {
        return Commands.runOnce(() -> preIntakeArmMotor.clearFaults());
    }

}
