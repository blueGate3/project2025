// Copyright (c) FIRST and other WPILib contributors. Coded by Jack McAllister, with a special shoutout to Mountain Dew (my unofficial sponsor)
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-swerve-aiming-and-ranging

package frc.robot.Subsystems;

import frc.robot.Constants;
import frc.robot.Constants.AlignConst;

public class AutoAlign {
  double x, y, rot, xSpeed, ySpeed, rotSpeed, desiredDegree;
  int tag;
  Drivetrain m_drivetrain;

  public AutoAlign(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
  }

  public double aimX(boolean leftBar) {
    if(leftBar) { //may need to flip the signs between the if/else depending on which side returns positive. 
      x = calculateXDistance() -.1;//+ Constants.X_SETPOINT_REEF_ALIGNMENT;
    } else {
      x = calculateXDistance() - Constants.X_SETPOINT_REEF_ALIGNMENT - .041; //calculate distance should return negative.
    }

    if(x < Constants.X_TOLERANCE_REEF_ALIGNMENT && x > -Constants.X_TOLERANCE_REEF_ALIGNMENT) { //if within tolerances
      x=0;
    }

    x *= Constants.X_REEF_ALIGNMENT_P;
    //x *= -1; //probably shouldn't need. 
    return x;
  }

  public double aimY() {
    y = (calculateYDistance()+.4); //delete constant later when we have time.

    y*= Constants.Y_REEF_ALIGNMENT_P;
    return y; //negative because back is forward with controllers
  }

  public double calculateXDistance() { //can return negative
    return calculateYDistance() * Math.tan(Math.toRadians(LimelightHelpers.getTX(""))); //use trig and you'll figure it out, this is the offset along the x axis from where we wanna be (distance from camera center to apriltag center)
    //returns meters.
  }
  public double calculateYDistance() {
    return Constants.LIMELIGHT_OFFSET / Math.tan(Math.toRadians(LimelightHelpers.getTY(""))); //returns meters. 
  }

  public double getTagRot(String limelightName) { //https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings-FieldLayoutAndMarking.pdf

    if(LimelightHelpers.getTV(limelightName)) {
      tag = (int) LimelightHelpers.getFiducialID("");
        switch (tag) { 
          case 10: case 21: desiredDegree = 1; break; 
          case 11: case 20: desiredDegree = 1; break; 
          case 6: case 19: desiredDegree = 1; break; 
          case 7: case 18: desiredDegree = -180; break; 
          case 8: case 17: desiredDegree = 1; break; 
          case 9: case 22: desiredDegree = 1; break; 
      }
    } else {
      System.out.println("No target! ");
    }
    System.out.println("Desired Degrees: " + desiredDegree);
    return desiredDegree;
  }
  public double aimRot(double desiredDegrees) {
    rot = desiredDegrees - m_drivetrain.getNavXHeading(); //may need to invert
    System.out.println("Navx Reading: " + m_drivetrain.getNavXHeading());
    rot *= Constants.ROT_REEF_ALIGNMENT_P;
    rot *= -1;
    if(rot < AlignConst.rotTol && rot > -AlignConst.rotTol) {
      rot=0;
    }
    return rot;
  }

  public double aimXHP() {
    x = calculateXDistance() - AlignConst.hpX;
  if(x < Constants.X_TOLERANCE_REEF_ALIGNMENT && x > -Constants.X_TOLERANCE_REEF_ALIGNMENT) { //if within tolerances
    x=0;
  }
  x *= Constants.X_REEF_ALIGNMENT_P;
  return x;
  }

  public void autoAlignReef(boolean leftBar) {
    if(LimelightHelpers.getTV(AlignConst.reefLimelight)) {
      xSpeed = aimX(leftBar);
      ySpeed = aimY();
      rotSpeed = aimRot(getTagRot(AlignConst.reefLimelight));
      m_drivetrain.drive(xSpeed, ySpeed, rotSpeed, false, false); //rot zero for now
    } else {
      System.out.println("No target for reef! ");
    }
  }

  public void autoAlignHP() {
    if(LimelightHelpers.getTV(AlignConst.hpLimelight)) {
      xSpeed = aimXHP();
      ySpeed = aimY();
      rotSpeed = aimRot(getTagRot(AlignConst.hpLimelight));
      m_drivetrain.drive(xSpeed, ySpeed, 0, false, false); //rot zero for now
    } else {
      System.out.println("No target for human player station! ");
    }
  }
}