// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    AddressableLED ledStrip;
    AddressableLEDBuffer ledBuffer;
    private final Distance ledSpacing;
    // LEDPattern pattern;

    public LEDSubsystem() {

        ledStrip = new AddressableLED(0);
        ledBuffer = new AddressableLEDBuffer(13);
        // pattern = LEDPattern.solid(Color.kAqua);
        ledStrip.setLength(ledBuffer.getLength());
        ledSpacing = Centimeters.of(19/13);

    }

    
    public void setColor(Color color) {
        LEDPattern pattern = LEDPattern.solid(color);
        pattern.applyTo(ledBuffer);
        ledStrip.setData(ledBuffer);
        ledStrip.start();
        // System.out.println("Set color");
    }

    public void blinkColor(Color color, Time time) {
        LEDPattern base = LEDPattern.solid(color);
        LEDPattern pattern = base.blink(time);
        pattern.applyTo(ledBuffer);
        ledStrip.setData(ledBuffer);
        ledStrip.start();
        // System.out.println("Set Blinking color");
    }

    public void blinkColor(Color color, Time onTime, Time offTime) {
        LEDPattern base = LEDPattern.solid(color);
        LEDPattern pattern = base.blink(onTime,offTime);
        pattern.applyTo(ledBuffer);
        ledStrip.setData(ledBuffer);
        ledStrip.start();
    }

    public void rainbowPattern(){
        LEDPattern pattern = LEDPattern.rainbow(255, 128);
        LEDPattern rainbowScroller = pattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.1), ledSpacing);
        rainbowScroller.applyTo(ledBuffer);
        ledStrip.setData(ledBuffer);
        ledStrip.start();
    }


    @Override
    public void periodic() {
    }
}