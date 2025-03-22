// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
public class Robot extends TimedRobot {

  RobotContainer m_RobotContainer = new RobotContainer();


  /**
 * Uses the CameraServer class to automatically capture video from a USB webcam and send it to the
 * FRC dashboard without doing any vision processing. This is the easiest way to get camera images
 * to the dashboard. Just add this to the robot class constructor.
 */
  public Robot() {
    //what we're trying now
    UsbCamera driverCam = CameraServer.startAutomaticCapture();
    driverCam.setResolution(1280, 720); //TODO look at other 16:9 resolutions
    driverCam.setFPS(60);
    m_RobotContainer.startLEDs();
  }

  @Override
  public void robotPeriodic() {
    //m_RobotContainer.temp();
    m_RobotContainer.runLEDs();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {m_RobotContainer.resetDriveEncoder();
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