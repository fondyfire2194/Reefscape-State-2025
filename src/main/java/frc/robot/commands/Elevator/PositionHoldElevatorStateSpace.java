// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utils.SD;

public class PositionHoldElevatorStateSpace extends Command {
    private final ElevatorSubsystem elevator;

    private boolean toggle;
    /*
     * The plant holds a state-space model of our elevator. This system has the
     * following properties:
     * 
     * States: [position, velocity], in meters and meters per second.
     * Inputs (what we can "put in"): [voltage], in volts.
     * Outputs (what we can measure): [position], in meters.
     * 
     * This elevator is driven by two NEO motors.
     */
    private final LinearSystem<N2, N1, N2> m_elevatorPlant;
    // The observer fuses our encoder data and voltage inputs to reject noise.
    private final KalmanFilter<N2, N1, N1> m_observer;

    // A LQR uses feedback to create voltage commands.
    private final LinearQuadraticRegulator<N2, N1, N1> m_controller;
    // The state-space loop combines a controller, observer, feedforward and plant
    // for easy control.
    private final LinearSystemLoop<N2, N1, N1> m_loop;

    @SuppressWarnings("unchecked")
    public PositionHoldElevatorStateSpace(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        m_elevatorPlant = LinearSystemId.createElevatorSystem(
                DCMotor.getNEO(2), elevator.kCarriageMass, elevator.kElevatorDrumRadiusMeters,
                elevator.kElevatorGearing);
        m_observer = new KalmanFilter<>(
                Nat.N2(),
                Nat.N1(),
                (LinearSystem<N2, N1, N1>) m_elevatorPlant.slice(0),
                VecBuilder.fill(Units.inchesToMeters(2), Units.inchesToMeters(20)),
                // How accurate we think our model is, in meters and meters/second.
                VecBuilder.fill(0.001), // How accurate we think our encoder position
                // data is. In this case we very highly trust our encoder position reading.
                0.020);
        m_controller = new LinearQuadraticRegulator<>(
                (LinearSystem<N2, N1, N1>) m_elevatorPlant.slice(0),
                VecBuilder.fill(Units.inchesToMeters(.5), Units.inchesToMeters(5.0)),
                // qelms. Position and velocity error tolerances, in meters and meters per
                // second.
                // Decrease this to more heavily penalize state excursion, or make the
                // controller behave more
                // aggressively. In this example we weight position much more highly than
                // velocity, but this can be tuned to balance the two.
                VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
                // heavily penalize control effort, or make the controller less aggressive. 12
                // is a good starting point because that is the (approximate) maximum voltage of
                // a battery.
                0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be

        m_loop = new LinearSystemLoop<>(
                (LinearSystem<N2, N1, N1>) m_elevatorPlant.slice(0),
                m_controller,
                m_observer,
                12.0,
                0.020);

        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
        double temp = elevator.getLeftPositionMeters();
        elevator.setGoalMeters(temp);
        // Reset our loop to make sure it's in a known state.
        m_loop.reset(VecBuilder.fill(elevator.getLeftPositionMeters(), elevator.getLeftVelocityMetersPerSecond()));

        // Reset our last reference to the current state.
        elevator.currentSetpoint = new TrapezoidProfile.State(elevator.getLeftPositionMeters(),
                elevator.getLeftVelocityMetersPerSecond());

    }

    @Override
    public void execute() {

        // Step our TrapezoidalProfile forward 20ms and set it as our next reference
        elevator.currentSetpoint = elevator.m_profile.calculate(0.020, elevator.currentSetpoint, elevator.m_goal);
        m_loop.setNextR(elevator.currentSetpoint.position, elevator.currentSetpoint.velocity);

        // Correct our Kalman filter's state vector estimate with encoder data.
        m_loop.correct(VecBuilder.fill(elevator.getLeftPositionMeters()));

        // Update our LQR to generate new voltage commands and use the voltages to
        // predict the next state with out Kalman filter.
        m_loop.predict(0.020);

        // Send the new calculated voltage to the motors.
        // voltage = duty cycle * battery voltage, so
        // duty cycle = voltage / battery voltage
        double nextVoltage = m_loop.getU(0);
        elevator.leftMotor.setVoltage(nextVoltage);

        toggle = !toggle;

        if (elevator.showTelemetry) {

            if (toggle) {
                SD.sd2("Elevator/SS/goalpos", elevator.m_goal.position);
                SD.sd2("Elevator/SS/position", elevator.getLeftPositionMeters());
            } else {

                SD.sd2("Elevator/SS/setvel", elevator.currentSetpoint.velocity);
                SD.sd2("Elevator/SS/setpos", elevator.currentSetpoint.position);
                SD.sd2("Elevator/SS/actvel", elevator.getLeftVelocityMetersPerSecond());

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