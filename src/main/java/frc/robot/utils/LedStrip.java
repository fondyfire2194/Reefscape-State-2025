// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants.Side;

/** Add your docs here. */
public class LedStrip {
    public AddressableLED m_led;
    public AddressableLEDBuffer m_ledbuffer;
    public AddressableLEDBufferView m_view1;
    public AddressableLEDBufferView m_view2;
    public AddressableLEDBufferView m_view3;
    public AddressableLEDBufferView m_view4;
    public AddressableLEDBufferView m_view5;
    public AddressableLEDBufferView m_view6;
    public AddressableLEDBufferView m_view7;

    public AddressableLEDSim m_LedSim;
    LEDPattern red = LEDPattern.solid(Color.kRed);
    LEDPattern green = LEDPattern.solid(Color.kGreen);
    LEDPattern yellow = LEDPattern.solid(Color.kYellow);
    LEDPattern blue = LEDPattern.solid(Color.kBlue);
    LEDPattern white = LEDPattern.solid(Color.kAntiqueWhite);
    LEDPattern coral_intake = LEDPattern.solid(Color.kGreen).blink(Seconds.of(0.2));
    LEDPattern coral_out = LEDPattern.solid(Color.kRed).blink(Seconds.of(0.2));

    LEDPattern fire;

    LEDPattern off = LEDPattern.solid(Color.kBlack);
    double intakeTimer = 0;

    public LedStrip() {
        m_led = new AddressableLED(0);

        m_ledbuffer = new AddressableLEDBuffer(60);
        m_led.setLength(m_ledbuffer.getLength());
        m_view1 = new AddressableLEDBufferView(m_ledbuffer, 1, 9);
        m_view2 = new AddressableLEDBufferView(m_ledbuffer, 10, 14);
        m_view3 = new AddressableLEDBufferView(m_ledbuffer, 15, 19);
        m_view4 = new AddressableLEDBufferView(m_ledbuffer, 20, 24);
        m_view5 = new AddressableLEDBufferView(m_ledbuffer, 25, 29);
        m_view6 = new AddressableLEDBufferView(m_ledbuffer, 30, 34);

        m_led.setData(m_ledbuffer);
        m_led.start();

        m_LedSim = new AddressableLEDSim(m_led);
        m_LedSim.setRunning(true);
        createLEDPatterns();
    }

    public void createLEDPatterns() {
        // Distance ledSpacing = Meters.of(1 / 35.0);
        LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kRed, Color.kOrange);
        fire = base.scrollAtRelativeSpeed(Percent.per(Seconds).of(200));

    }

    public void setViewOneSolidColor(int reefZone) {
        double currentTime = System.currentTimeMillis() / 1000;
        if ((currentTime - intakeTimer) > 1) {
            switch (reefZone) {
                case 0:
                    off.applyTo(m_ledbuffer);
                    break;
                case 1:
                    blue.applyTo(m_ledbuffer);
                    break;
                case 2:
                    red.applyTo(m_ledbuffer);
                    break;
                case 3:
                    blue.applyTo(m_ledbuffer);
                    break;
                case 4:
                    red.applyTo(m_ledbuffer);
                    break;
                case 5:
                    blue.applyTo(m_ledbuffer);
                    break;
                case 6:
                    red.applyTo(m_ledbuffer);
                    break;
                default:
                    off.applyTo(m_ledbuffer);
                    break;
            }
            m_led.setData(m_ledbuffer);
        }
    }

    public void setViewTwoSolidColor(Side side) {
        int n = side.ordinal();
        switch (n) {
            case 0:
                red.applyTo(m_view2);
                break;
            case 1:
                blue.applyTo(m_view2);
                break;
            case 2:
                yellow.applyTo(m_view2);
                break;
            default:
                off.applyTo(m_view2);
                break;
        }
        m_led.setData(m_ledbuffer);
    }

    public void setViewThreeSolidColor(int setpoint) {

        switch (setpoint) {
            case 0:
                off.applyTo(m_view3);
                break;
            case 1:
                white.applyTo(m_view3);
                break;
            case 2:
                green.applyTo(m_view3);
                break;
            case 3:
                yellow.applyTo(m_view3);
                break;
            case 4:
                red.applyTo(m_view3);
                break;
            default:
                off.applyTo(m_view1);
                break;

        }

        m_led.setData(m_ledbuffer);

    }

    public Command getCoralIntakeLEDsCommand() {
        return Commands.parallel(Commands.runOnce(() -> intakeTimer = System.currentTimeMillis() / 1000),
                Commands.run(() -> coral_intake.applyTo(m_ledbuffer)),
                Commands.run(() -> m_led.setData(m_ledbuffer))).withTimeout(1);
    }

    public Command getCoralDeliverLEDsCommand() {
        return Commands.parallel(Commands.runOnce(() -> intakeTimer = System.currentTimeMillis() / 1000),
                Commands.run(() -> fire.applyTo(m_ledbuffer)),
                Commands.run(() -> m_led.setData(m_ledbuffer))).withTimeout(1);
    }
}
