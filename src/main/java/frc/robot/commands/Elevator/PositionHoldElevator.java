// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utils.SD;

public class PositionHoldElevator extends Command {
    private final ElevatorSubsystem elevator;

    private PIDController pidController;
    private double kp = 25.;
    private double ki = 0;
    private double kd = 0.;
    private double tolerance = Units.inchesToMeters(1);

    public PositionHoldElevator(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        pidController = new PIDController(kp, ki, kd);
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
        pidController.disableContinuousInput();
        pidController.setTolerance(tolerance);
        double temp = elevator.getLeftPositionMeters();
        elevator.setGoalMeters(temp);
        if (elevator.showTelemetry)
            SmartDashboard.putData(" Elevator/PID/controller1", pidController);
    }

    @Override
    public void execute() {

        elevator.nextSetpoint = elevator.m_profile.calculate(.02, elevator.currentSetpoint, elevator.m_goal);

        elevator.leftff = elevator.eff.calculate(elevator.nextSetpoint.velocity);
        double accel = (elevator.nextSetpoint.velocity - elevator.currentSetpoint.velocity) * elevator.elevatorKa * 50;
        double pidout =0;// pidController.calculate(elevator.getLeftPositionMeters(), elevator.nextSetpoint.position);

        elevator.currentSetpoint = elevator.nextSetpoint;
        double ff = elevator.leftff + accel + pidout;
        if (elevator.showTelemetry) {
            SD.sd2("Elevator/FF/ff", ff);
            SD.sd2("Elevator/FF/setpos", elevator.currentSetpoint.position);
            SD.sd2("Elevator/FF/setvel", elevator.currentSetpoint.velocity);
            SD.sd2("Elevator/FF/actvel", elevator.getLeftVelocityMetersPerSecond());
            SD.sd2("Elevator/FF/actpos", elevator.getLeftPositionMeters());

        }

        elevator.leftMotor.setVoltage(ff);
    }

    public void runAtVelocity(double metersPerSecond) {
        elevator.leftClosedLoopController.setReference(metersPerSecond, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}