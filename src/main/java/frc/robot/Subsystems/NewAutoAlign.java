package frc.robot.Subsystems;

import frc.robot.Constants;
import frc.robot.Constants.AlignConst;

public class NewAutoAlign {
  double x, y, rot, xSpeed, ySpeed, rotSpeed, desiredDegree;
  Drivetrain m_drivetrain;

  public NewAutoAlign(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
  }

  public double aimX(boolean leftBar) {
    if(leftBar) { //may need to flip the signs between the if/else depending on which side returns positive. 
      x = calculateXDistance() +.04 ;//+ Constants.X_SETPOINT_REEF_ALIGNMENT;
    } else {
      x = calculateXDistance() - Constants.X_SETPOINT_REEF_ALIGNMENT - .10; //calculate distance should return negative.
    }
    if(x < Constants.X_TOLERANCE_REEF_ALIGNMENT && x > -Constants.X_TOLERANCE_REEF_ALIGNMENT) { //if within tolerances
      x=0;
    }
    x *= Constants.X_REEF_ALIGNMENT_P;
    return x;
  }

  public double aimY() {
    y = (calculateYDistance()+.4); //delete constant later when we have time? I think .4 works pretty well, maybe .35
    y*= Constants.Y_REEF_ALIGNMENT_P;
    return -y; //negative because back is forward with controllers
  }

  public double calculateXDistance() { //can return negative
    return calculateYDistance() * Math.tan(Math.toRadians(LimelightHelpers.getTX(""))); //use trig and you'll figure it out, this is the offset along the x axis from where we wanna be (distance from camera center to apriltag center)
    //returns meters.
  }
  public double calculateYDistance() {
    return Constants.LIMELIGHT_OFFSET / Math.tan(Math.toRadians(LimelightHelpers.getTY(""))); //returns meters. 
  }

  
  public void align(boolean leftBar) { //for now this only lines up the x, not the y. The y and rot will be taken care of in the approach function
    if(LimelightHelpers.getTV("")) {
      xSpeed = aimX(leftBar);
      m_drivetrain.drive(xSpeed, 0, 0, false, false); //rot zero for now
    } else {
      System.out.println("No target for reef! ");
    }
  }

  public double lineupX() {
    return (calculateXDistance()-.1) * Constants.X_REEF_ALIGNMENT_P; //where we want to be - where we are * proportion
  }

  public double lineupRot() {
    return (LimelightHelpers.getTX("")) * Constants.ROT_REEF_ALIGNMENT_P; //where we want to be - where we are * proportion
  }

  public void approach() { //gets us directly in line with target. hopefully. 
    if(LimelightHelpers.getTV("")) {
        xSpeed = lineupX();
        ySpeed = aimY();
        rotSpeed = lineupRot();
        m_drivetrain.drive(xSpeed, ySpeed, 0, false, false); //hopefully add in rotation later 
    } else {
        System.out.println("No target for reef! ");
    }
  }
}