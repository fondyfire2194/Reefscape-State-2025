// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Factories.CommandFactory.Setpoint;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utils.SD;

public class PositionHoldElevatorExponential extends Command {
    private final ElevatorSubsystem elevator;
    private final ExponentialProfile m_profile;

    private ExponentialProfile.State m_setpoint = new ExponentialProfile.State(0, 0);

    private PIDController pidController;
    private double kp = 25.;
    private double ki = 0;
    private double kd = 0.;

    private double tolerance = Units.inchesToMeters(1);

    private boolean toggle;

    public PositionHoldElevatorExponential(ElevatorSubsystem elevator) {
        this.elevator = elevator;

        pidController = new PIDController(kp, ki, kd);
        m_profile = new ExponentialProfile(
                ExponentialProfile.Constraints.fromCharacteristics(
                        12, elevator.elevatorKv, elevator.elevatorKa));
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {

        pidController.disableContinuousInput();
        pidController.setTolerance(tolerance);
        double temp = elevator.getLeftPositionMeters();
        elevator.setGoalMeters(temp);
        if (elevator.showTelemetry)
            SmartDashboard.putData(" Elevator/EXP/controller", pidController);
    }

    @Override
    public void execute() {

        toggle = !toggle;

        var goalState = new ExponentialProfile.State(elevator.m_goal.position, 0);
        if (elevator.showTelemetry) {
            if (toggle) {
                SD.sd2("Elevator/EXP/goalstatepos", goalState.position);
                SD.sd2("Elevator/EXP/goalstatevel", goalState.velocity);
            }
        }
        var next = m_profile.calculate(0.020, m_setpoint, goalState);
        if (elevator.showTelemetry) {
            if (!toggle) {
                SD.sd2("Elevator/EXP/nextstatepos", next.position);
                SD.sd2("Elevator/EXP/nextstatevel", next.velocity);
                SD.sd2("Elevator/EXP/nextleftposmeteers", elevator.getLeftPositionMeters());
            }
        }
        // With the setpoint value we run PID control like normal
        double pidOutput = pidController.calculate(elevator.getLeftPositionMeters(), m_setpoint.position);
        // double feedforward =
        // elevator.eff.calculateWithVelocities(m_setpoint.velocity, next.velocity);
        double feedforward = elevator.eff.calculate(next.velocity);
        double accel = (next.velocity - m_setpoint.velocity) * elevator.elevatorKa * 50;
        elevator.leftMotor.setVoltage(pidOutput + feedforward + accel);

        m_setpoint = next;

        if (elevator.showTelemetry) {

            if (toggle) {
                SD.sd2("Elevator/EXP/goalpos", elevator.m_goal.position);
                SD.sd2("Elevator/EXP/actpos", elevator.getLeftPositionMeters());
                SD.sd2("Elevator/EXP/nextpos", next.position);
                SD.sd2("Elevator/EXP/setpos", m_setpoint.position);

                SD.sd2("Elevator/EXP/poserror", pidController.getError());

            } else {

                SD.sd2("Elevator/EXP/nextvel", next.velocity);
                SD.sd2("Elevator/EXP/setvel", m_setpoint.velocity);

                SD.sd2("Elevator/EXP/feedfor", feedforward);
                SD.sd2("Elevator/EXP/pidout", pidOutput);
                SD.sd2("Elevator/EXP/actvel", elevator.getLeftVelocityMetersPerSecond());
                SmartDashboard.putBoolean("Elevator/EXP/atSetpoint", pidController.atSetpoint());
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