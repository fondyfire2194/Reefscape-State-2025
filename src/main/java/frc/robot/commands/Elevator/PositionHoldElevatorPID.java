// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utils.SD;

public class PositionHoldElevatorPID extends Command {
    private final ElevatorSubsystem elevator;

    private PIDController pidController;
    private double kp = 5.;
    private double ki = 0;
    private double kd = 0.;
    private double izone = .5;
    private double minIntegral = -.1;
    private double maxIntegral = .1;
    private double tolerance = Units.inchesToMeters(1);
    private double maxuprate = 5;
    private double maxdownrate = 2;

    private boolean toggle;

    private double ffGain = .2;

    public PositionHoldElevatorPID(ElevatorSubsystem elevator) {
        this.elevator = elevator;

        pidController = new PIDController(kp, ki, kd);
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

        elevator.nextSetpoint = elevator.m_profile.calculate(.02, elevator.currentSetpoint, elevator.m_goal);

        double mps = pidController.calculate(elevator.getLeftPositionMeters(), elevator.nextSetpoint.position);

        mps += elevator.nextSetpoint.velocity * ffGain;

        double kgmps = 1.2 * elevator.elevatorKg / elevator.elevatorKv;

        mps += kgmps;

        if (elevator.showTelemetry) {

            if (toggle) {
                SD.sd2("Elevator/PID/goalpos", elevator.m_goal.position);
                SD.sd2("Elevator/PID/currsetpos", elevator.currentSetpoint.position);
                SD.sd2("Elevator/PID/setpos", elevator.nextSetpoint.position);
                SD.sd2("Elevator/PID/actpos", elevator.getLeftPositionMeters());

            } else {
                SD.sd2("Elevator/PID/setvel", elevator.nextSetpoint.velocity);
                SD.sd2("Elevator/PID/currsetvel", elevator.currentSetpoint.velocity);
                SD.sd2("Elevator/PID/actmps", elevator.getLeftVelocityMetersPerSecond());
                SD.sd2("Elevator/PID/poserror", pidController.getError());
                SmartDashboard.putBoolean("Elevator/PID/atSetpoint", pidController.atSetpoint());
            }
        }
        mps = MathUtil.clamp(mps, -maxdownrate, maxuprate);

        if (elevator.showTelemetry)
            SD.sd2("Elevator/PID/mpsclamped", mps);

        elevator.runAtVelocity(mps);

        elevator.currentSetpoint = elevator.nextSetpoint;

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}