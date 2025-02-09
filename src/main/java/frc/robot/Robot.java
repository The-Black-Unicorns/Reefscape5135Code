// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import org.w3c.dom.css.RGBColor;

import com.ctre.phoenix6.signals.Led1OffColorValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color.RGBChannel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private LEDSubsystem ledSubsystem;
  private SwerveSubsystem swerveSubsystem;
  StructPublisher<Pose2d> ppublisher;

  public Robot() {
    m_robotContainer = new RobotContainer();
    ledSubsystem = new LEDSubsystem();
    swerveSubsystem = new SwerveSubsystem();
    ppublisher = NetworkTableInstance.getDefault().getStructTopic("Pose", Pose2d.struct).publish();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    ledSubsystem.setColor(Color.kRed);
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    ledSubsystem.blinkColor(Color.kGreen, Seconds.of(0.4));
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    ppublisher.set(swerveSubsystem.getPose());
    // ledSubsystem.blinkColor(new Color(87, 27, 126), Seconds.of(0.1));
    ledSubsystem.setColor(new Color(87, 27, 126));
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    
  }

  @Override
  public void testPeriodic() {
    ledSubsystem.rainbowPattern();
  }

  @Override
  public void testExit() {}
}
