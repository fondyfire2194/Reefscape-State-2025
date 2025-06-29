// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utils.SD;

public class PositionHoldElevatorExponential extends Command {
    private final ElevatorSubsystem elevator;
    private final ExponentialProfile m_profile;

    private ExponentialProfile.State m_setpoint = new ExponentialProfile.State(0, 0);

    private PIDController pidController;
    private double kp = 20.;
    private double ki = 0;
    private double kd = 0.2;
    private double izone = .5;
    private double minIntegral = -.1;
    private double maxIntegral = .1;
    private double tolerance = Units.inchesToMeters(1);

    private boolean toggle;

    public PositionHoldElevatorExponential(ElevatorSubsystem elevator) {
        this.elevator = elevator;

        pidController = new PIDController(kp, ki, kd);
        m_profile = new ExponentialProfile(
                ExponentialProfile.Constraints.fromCharacteristics(
                        10, elevator.elevatorKv, elevator.elevatorKa));
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {

        pidController.setIZone(izone);
        pidController.disableContinuousInput();
        pidController.setIntegratorRange(minIntegral, maxIntegral);
        pidController.setTolerance(tolerance);
        double temp = elevator.getLeftPositionMeters();
        elevator.setGoalMeters(temp);
        if (elevator.showTelemetry)
            SmartDashboard.putData(" Elevator/PID/controller", pidController);
    }

    @Override
    public void execute() {

        toggle = !toggle;

        var goalState = new ExponentialProfile.State(elevator.m_goal.position, 0);
        SD.sd2("Elevator/goalstatepos",  goalState.position);
        SD.sd2("Elevator/goalstatevel",  goalState.velocity);

        var next = m_profile.calculate(0.020, m_setpoint, goalState);
        SD.sd2("Elevator/nextstatepos",  next.position);
        SD.sd2("Elevator/nextstatevel",  next.velocity);
        SD.sd2("Elevator/nextleftposmeteers",  elevator.getLeftPositionMeters());

        // With the setpoint value we run PID control like normal
        double pidOutput = pidController.calculate(elevator.getLeftPositionMeters(), m_setpoint.position);
        double feedforwardOutput = elevator.eff.calculateWithVelocities(m_setpoint.velocity, next.velocity);

        elevator.leftMotor.setVoltage(pidOutput + feedforwardOutput);

        m_setpoint = next;

        if (elevator.showTelemetry) {

            if (toggle) {
                SmartDashboard.putNumber("Elevator/PID/goalpos", elevator.m_goal.position);

            } else {
                SmartDashboard.putNumber("Elevator/PID/position", elevator.getLeftPositionMeters());
                SmartDashboard.putNumber("Elevator/PID/setvel", elevator.nextSetpoint.velocity);
                SmartDashboard.putNumber("Elevator/PID/setpos", Units.metersToInches(elevator.nextSetpoint.position));

                SmartDashboard.putNumber("Elevator/PID/mpsRead", elevator.getLeftVelocityMetersPerSecond());
                SmartDashboard.putNumber("Elevator/PID/poserror", pidController.getError());
                SmartDashboard.putBoolean("Elevator/PID/atSetpoint", pidController.atSetpoint());
            }
        }

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}