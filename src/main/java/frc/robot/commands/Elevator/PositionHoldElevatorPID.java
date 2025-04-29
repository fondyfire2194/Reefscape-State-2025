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
    private double maxplusrate = 3;
    private double maxminusrate = 1;

    private boolean toggle;

    private double ffgain = .3;

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

        toggle = !toggle;// split display for time purposes

        elevator.nextSetpoint = elevator.m_profile.calculate(.02, elevator.currentSetpoint, elevator.m_goal);

        double radpersec = pidController.calculate(elevator.getLeftPositionMeters(), elevator.nextSetpoint.position);

        double nextVel = elevator.nextSetpoint.velocity;

        double ksvmps = (elevator.elevatorKs) * Math.signum(nextVel) / elevator.elevatorKv;

        double kgmps = elevator.elevatorKg / elevator.elevatorKv;

        double accel = (elevator.currentSetpoint.velocity - elevator.nextSetpoint.velocity) * 50;

        double accelmps = (accel * elevator.elevatorKa) / elevator.elevatorKv;

        double velff = elevator.nextSetpoint.velocity * ffgain;

        double mpstotal = radpersec + velff + kgmps + ksvmps + accelmps;

        double mpsclamped = MathUtil.clamp(mpstotal, -maxminusrate, maxplusrate);

        elevator.runAtVelocity(mpsclamped);

        elevator.currentSetpoint = elevator.nextSetpoint;
        
        if (elevator.showTelemetry) {
            if (toggle) {
                SmartDashboard.putNumber("Elevator/Trap/goalpos", Units.metersToInches(elevator.m_goal.position));
                SmartDashboard.putNumber("Elevator/Trap/currsetpos", Units.metersToInches(elevator.currentSetpoint.position));
                SmartDashboard.putNumber("Elevator/Trap/currsetvel", Units.metersToInches(elevator.currentSetpoint.velocity));
                SmartDashboard.putNumber("Elevator/Trap/setpos", Units.metersToInches(elevator.nextSetpoint.position));
            } else {
                SmartDashboard.putNumber("Elevator/Trap/setvel", Units.metersToInches(elevator.nextSetpoint.velocity));
                SmartDashboard.putNumber("Elevator/Trap/degpersec", Units.metersToInches(radpersec));
                SmartDashboard.putNumber("Elevator/Trap/poserror", Units.metersToInches(pidController.getError()));
                SmartDashboard.putBoolean("Elevator/Trap/atsetpoint", pidController.atSetpoint());
                SmartDashboard.putNumber("Elevator/Trap/mpsclamped", Units.metersToInches(mpsclamped));
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