// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
public class Robot extends TimedRobot {

  RobotContainer m_RobotContainer = new RobotContainer();
  private Spark m_ledSpark;

  /**
 * Uses the CameraServer class to automatically capture video from a USB webcam and send it to the
 * FRC dashboard without doing any vision processing. This is the easiest way to get camera images
 * to the dashboard. Just add this to the robot class constructor.
 */
  public Robot() {
    UsbCamera m_Camera = CameraServer.startAutomaticCapture();
    m_Camera.setFPS(60);
    m_Camera.setResolution(1080, 720); //todo look up proper ratios.
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_RobotContainer.AutoStart();
  }

  @Override
  public void autonomousPeriodic() {
    m_RobotContainer.Auto();
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    m_RobotContainer.drive();
    m_RobotContainer.operate();
    m_RobotContainer.runLimelightLED();
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