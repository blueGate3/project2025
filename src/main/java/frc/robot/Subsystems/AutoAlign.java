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
import frc.robot.Constants.AlignConst;
import frc.robot.Subsystems.LimelightHelpers;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Constants;

public class AutoAlign {
  double x, y, rot;


  public AutoAlign() {

  }
  public double alignX(boolean leftBar) {
    if(leftBar) {
      x = calculateXDistance() + Constants.X_SETPOINT_REEF_ALIGNMENT; //you double minus because of the way it will return, look at picture from 3/30/2025 (or ask jack mcallister to send it to you if you're not me)
    } else {
      x = calculateXDistance() - Constants.X_SETPOINT_REEF_ALIGNMENT;
    }
    x *= Constants.X_REEF_ALIGNMENT_P;
    if(x < Constants.X_TOLERANCE_REEF_ALIGNMENT || x > -Constants.X_TOLERANCE_REEF_ALIGNMENT) { //if within tolerances
      x=0;
    }
    return x;
  }

  public double alignY() {
    y = (calculateYDistance() - Constants.Y_SETPOINT_REEF_ALIGNMENT) * Constants.Y_REEF_ALIGNMENT_P;
    if (y < Constants.Y_TOLERANCE_REEF_ALIGNMENT || y > -Constants.Y_TOLERANCE_REEF_ALIGNMENT) {
      y = 0;
    }
    return y;
  }

  public double aimRot() {
    return 1;//temp so no errors
  }

  public double calculateXDistance() { //can return negative
    return calculateYDistance() * Math.tan(Math.toRadians(LimelightHelpers.getTX(""))); //use trig and you'll figure it out, this is the offset along the x axis from where we wanna be (distance from camera center to apriltag center)
    //returns meters.
  }
  public double calculateYDistance() {
    return Constants.LIMELIGHT_OFFSET * Math.tan(Math.toRadians(LimelightHelpers.getTY(""))); //returns meters. 
  }

  //TODO DOUBLE CHECK MATH, USE WHITEBOARD PICTURE, MAKE SURE TAN IS OKAY (DISTANCE GETTERS CAN RETURN NEGATIVE), THEN ACTUALLY SLAP A DRIVE FUNCTION ON THIS THING
  //ALSO ADD ROTATION, THIS IS JUST X AND Y
  //U GOT THIS JACK U FRICKIN ROCK
}