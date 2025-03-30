// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.LimelightHelpers;
import frc.robot.Subsystems.Drivetrain;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Drivetrain m_drivetrain;
  private double tagID = -1;

  public AlignToReefTagRelative(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    xController = new PIDController(Constants.X_REEF_ALIGNMENT_P, 0.0, 0);  // Vertical movement
    yController = new PIDController(Constants.Y_REEF_ALIGNMENT_P, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(Constants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
  }

  public void AutoAlignStart(boolean isRightScore) {
    xController.setSetpoint(isRightScore ? Constants.X_SETPOINT_REEF_ALIGNMENT : -Constants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(Constants.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("");
  }

  public void AutoAlignPeriodic() {
    if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID) {

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("");

      double xSpeed = xController.calculate(postions[2]);
      //double ySpeed = -yController.calculate(postions[0]);
      //double rotValue = -rotController.calculate(postions[4]);

      //m_drivetrain.drive(-xSpeed, 0, 0, false, false);

      if (!rotController.atSetpoint()) {
            m_drivetrain.drive(xSpeed, 0, 0, false, false); //maybe this should be here instead?
      }
    } else {
      m_drivetrain.drive(0, 0, 0, false, false);
    }
  }


}