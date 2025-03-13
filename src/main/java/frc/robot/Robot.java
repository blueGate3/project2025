// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.cameraserver.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
public class Robot extends TimedRobot {

  RobotContainer m_RobotContainer = new RobotContainer();
  private Command m_autonomousCommand;

  /**
 * Uses the CameraServer class to automatically capture video from a USB webcam and send it to the
 * FRC dashboard without doing any vision processing. This is the easiest way to get camera images
 * to the dashboard. Just add this to the robot class constructor.
 */
  public Robot() {
    // CameraServer.putVideo(null, 0, 0);
    CameraServer.startAutomaticCapture().setResolution(720, 500);
    //CameraServer.getVideo().getSource().setFPS(60);
    //CameraServer.putVideo("DriverFeed", 200, 500);

    // Creates UsbCamera and MjpegServer [1] and connects them
    // Creates the CvSink and connects it to the UsbCamera
    // CvSink cvSink = CameraServer.getVideo();
    // Creates the CvSource and MjpegServer [2] and connects them
    // CvSource outputStream = CameraServer.putVideo("Blur", 640, 480);
  }

  @Override
  public void robotPeriodic() {
    //m_RobotContainer.temp();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    
    // FollowPathCommand.warmupCommand().schedule();
    // m_autonomousCommand = m_RobotContainer.getAutonomousCommand();
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }

    
    m_RobotContainer.resetDriveEncoder();
    m_RobotContainer.resetTimer();
    m_RobotContainer.startTimer();
  }

  @Override
  public void autonomousPeriodic() {
    //m_autonomousCommand.schedule();
    m_RobotContainer.autopath();
  }

  @Override
  public void autonomousExit() {
    m_RobotContainer.stopTimer();
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    m_RobotContainer.readDriverController();
    m_RobotContainer.letDriverCook();
    m_RobotContainer.letOperatorCookUpdated();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}