// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-swerve-aiming-and-ranging

package frc.robot.Subsystems;

import java.lang.constant.Constable;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.LimelightHelpers;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Constants;

public class AutoAlign {
  double x, y, rot, xSpeed, ySpeed, rotSpeed;
  Drivetrain m_drivetrain;

  public AutoAlign(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
  }

  public double aimX(boolean leftBar) {
    System.out.println("X distance: " +calculateXDistance());
    System.out.println("Y distance: " +calculateYDistance());
    if(leftBar) { //may need to flip the signs between the if/else depending on which side returns positive. 
      x = calculateXDistance() ;//+ Constants.X_SETPOINT_REEF_ALIGNMENT;
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
    y = (calculateYDistance()+.1);
    // if(y< Constants.Y_TOLERANCE_REEF_ALIGNMENT) {
    //   y=0;
    // }

    y*= Constants.Y_REEF_ALIGNMENT_P;
    return -y; //negative because back is forward with controllers
  }

  public double alignRot() {
    return (LimelightHelpers.getTXNC("") - Constants.ROT_SETPOINT_REEF_ALIGNMENT) * Constants.ROT_REEF_ALIGNMENT_P;
  }

  public double calculateXDistance() { //can return negative
    return calculateYDistance() * Math.tan(Math.toRadians(LimelightHelpers.getTX(""))); //use trig and you'll figure it out, this is the offset along the x axis from where we wanna be (distance from camera center to apriltag center)
    //returns meters.
  }
  public double calculateYDistance() {
    return Constants.LIMELIGHT_OFFSET / Math.tan(Math.toRadians(LimelightHelpers.getTY(""))); //returns meters. 
  }

  public void autoAlign(boolean leftBar) {
    //put function so that we only run this if we have a limelight.
    if(LimelightHelpers.getTV("")) {
      xSpeed = aimX(leftBar);
      ySpeed = aimY();
      m_drivetrain.drive(0, 0, alignRot(), false, false); //rot zero for now
    } else {
      System.out.println("No target! ");
    }
  }

  //TODO DOUBLE CHECK MATH, USE WHITEBOARD PICTURE, MAKE SURE TAN IS OKAY (DISTANCE GETTERS CAN RETURN NEGATIVE), THEN ACTUALLY SLAP A DRIVE FUNCTION ON THIS THING
  //ALSO ADD ROTATION, THIS IS JUST X AND Y
  //U GOT THIS JACK U FRICKIN ROCK
}