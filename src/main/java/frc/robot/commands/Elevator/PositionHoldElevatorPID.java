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
    private double kp = 10.;
    private double ki = 0;
    private double kd = 0.2;
    private double izone = .5;
    private double minIntegral = -.1;
    private double maxIntegral = .1;
    private double tolerance = Units.inchesToMeters(1);
    private double maxuprate = 3;
    private double maxdownrate = 1;

    private boolean toggle;

    private double ffGain = .3;

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

        double velff = elevator.nextSetpoint.velocity * ffGain;

        if (toggle && elevator.showTelemetry) {
            SD.sd2("Elevator/PID/mps", mps);
            SD.sd2("Elevator/PID/vff", velff);
            SD.sd2("Elevator/PID/nextvel", elevator.nextSetpoint.velocity);
        }

        double nextVel = elevator.nextSetpoint.velocity;

        double ksvmps = (elevator.elevatorKs * Math.signum(nextVel)) / elevator.elevatorKv;

        double kgmps = elevator.elevatorKg / elevator.elevatorKv;

        if (toggle && elevator.showTelemetry)
            SD.sd2("Elevator/PID/ksv", ksvmps);

        double accel = (elevator.currentSetpoint.velocity - elevator.nextSetpoint.velocity) * 50;

        double accelmps = (accel * elevator.elevatorKa) / elevator.elevatorKv;

        double mpstotal = mps + velff + ksvmps + kgmps + accelmps;

        double mpsclamped = MathUtil.clamp(mpstotal, -maxdownrate, maxuprate);

        if (!toggle && elevator.showTelemetry) {
            SD.sd2("Elevator/PID/mpsclamped", mpsclamped);
            SD.sd2("Elevator/PID/actvel", elevator.getLeftVelocityMetersPerSecond());
            SD.sd2("Elevator/PID/actpos", elevator.getLeftPositionMeters());
        }

        elevator.runAtVelocity(mpsclamped);

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