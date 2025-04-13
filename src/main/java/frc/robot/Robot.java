// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cameraserver.CameraServerShared;
import edu.wpi.first.cameraserver.CameraServerSharedStore;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends TimedRobot {
  public static final String CTREConfigs = null;

public static Object ctreConfigs;
private Command m_autonomousCommand;
private DoublePublisher matchtime;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    
    // DataLogManager.start();
    // DriverStation.startDataLog(DataLogManager.getLog());

    // SmartDashboard.putNumber("My Field", 3.14);
    // CameraServer.startAutomaticCapture();

    PathfindingCommand.warmupCommand().schedule();
    
    matchtime = NetworkTableInstance.getDefault()
      .getDoubleTopic("CurrentMatchTime").publish();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.periodic();
    matchtime.set(DriverStation.getMatchTime());
    
    
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    m_robotContainer.enabledInit();

  }

  @Override
  public void disabledExit() {
    
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    m_robotContainer.enabledInit();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.enabledInit();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.enabledInit();
  }

  @Override
  public void testPeriodic() {
    m_robotContainer.testPeriodic();
  }

  @Override
  public void testExit() {}
}