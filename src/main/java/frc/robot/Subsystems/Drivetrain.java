// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;
import com.studica.frc.AHRS;
import com.thethriftybot.Conversion;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
//import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a swerve drive style drivetrain. */

public class Drivetrain extends SubsystemBase {

    // kMaxSpeed was 2 AND kmaxangularspeed was pi/3 (before testing [district champs])
    // SOLID SPEEDS 3.25 M/S /AND PI/2.25 ROT/S
    public static final double kMaxSpeed = 3.25; // 3.68 meters per second or 12.1 ft/s (max speed of SDS Mk3 with Neo motor) TODO change max speed coz hellllll yea
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    int invert = 1; //this will change depending on the alliance we are put on, it will be multiplied by -1 if we are red alliance and then multiplied by all of the drive inputs so we still drive the correct way and can remain blue alliance oriented for apriltags. 
    //more information can be found at https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html 

    private final AHRS navx = new AHRS(null); //TODO need to know

    // Locations of each swerve module relative to the center of the robot
    private final Translation2d m_frontRightLocation = new Translation2d( 0.37465, -0.37465);//side length total is at 29.5 inches including modules. Divided by 2 and set to meters is .37465 meters from one side to the tip of the module
    private final Translation2d m_frontLeftLocation = new Translation2d(0.37465,  0.37465);//the frc kinematics section has the coordinates so x is front-back, where front is positive, and y is left-right, where left is positive. it's communist to the extreme but will affect the way we initialize our module locations.
    private final Translation2d m_backLeftLocation = new Translation2d(-0.37465,  0.37465);//continued: that's the reason for the strange abnormal abhorrent disgusting affronts-before-God translation signs. 
    private final Translation2d m_backRightLocation = new Translation2d( -0.37465, -0.37465); //Try making this double negative and the one above it a y-positive only if it isnt aligned. also try doubling it if it's still off, thats what last year has it as

    // Constructor for each swerve module
    //Turn encoder values run through NAVX ports last year iirc, the port numbers dont match with what's printed so we have to run it through 
    private final SwerveModule m_frontRight = new SwerveModule(1, 2, 12, 0, false, false); //
    private final SwerveModule m_frontLeft = new SwerveModule(3, 4, 13, 0, false, false); //
    private final SwerveModule m_backLeft = new SwerveModule(5, 6, 18, 0, false, false); //
    private final SwerveModule m_backRight = new SwerveModule(7, 8, 20, 0, false, false); //
    //note, it is possible that we will need to change all of these channels, if this is the case then according to https://docs.revrobotics.com/brushless/spark-max/encoders/absolute, we will need to change all ports to 6 //probably not
    //TODO must work on motor inversion.

    // Swerve Drive Kinematics (note the ordering [frontRight, frontLeft, backLeft, backRight] [counterclockwise from the frontRight])
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontRightLocation, m_frontLeftLocation, m_backLeftLocation, m_backRightLocation);

    //INITIAL POSITIONS to help define swerve drive odometry. THis was a headache
    public SwerveModulePosition positionFrontLeft = new SwerveModulePosition(0.0,new Rotation2d(0.0));
    public SwerveModulePosition positionFrontRight = new SwerveModulePosition(0.0,new Rotation2d(0.0));
    public SwerveModulePosition positionBackLeft = new SwerveModulePosition(0.0,new Rotation2d(0.0));
    public SwerveModulePosition positionBackRight = new SwerveModulePosition(0.0,new Rotation2d(0.0));
    public SwerveModulePosition[] initialPositions = {positionFrontRight, positionFrontLeft, positionBackLeft, positionBackRight};
    public SwerveDriveKinematics m_initialStates; 
    public SwerveModulePosition[] positions = new SwerveModulePosition[4];
    
    public final SwerveDriveOdometry m_odometry;
    public final SwerveModule Swerves[] = {m_frontRight, m_frontLeft, m_backLeft, m_backRight};
    
    // Constructor
    public Drivetrain() {
        m_initialStates = new SwerveDriveKinematics(m_frontRightLocation, m_frontLeftLocation, m_backLeftLocation, m_backRightLocation);

        m_odometry = new SwerveDriveOdometry(
            m_kinematics, 
            navx.getRotation2d(), initialPositions
        );
        var alliance = DriverStation.getAlliance(); //see information where we set up the invert integer. 
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            invert = -1;
        }
        // m_visionSubsystem = new VisionSubsystem();
        // Configure AutoBuilder last i added all this in.
  }

    @Override
    public void periodic () {
        updateOdometry();
        //SmartDashboard.putNumber("xOdometry", getCurrentPose2d().getX());

        double gyroAngle = navx.getAngle();
        
    }

    /**
     * Update the odometry and vision poses
     */
    public void setPose (Pose2d pose) {
        m_odometry.resetPosition(new Rotation2d(pose.getRotation().getRadians()), initialPositions, pose);
        }

    /**
     * Takes in the current position of the robot, then figures out the distance to the center of the reef, and returns the center of rotation as the center of the reef
     * @param currentPose the current position of the robot.
     * @param allianceColor boolean for determining alliance color, as of now blue is false and red is true (to invert).
     */
    public Translation2d getPoseToReefCenter(Pose2d currentPose, boolean allianceColor) {

        /**
         * Notes about coordinate system: First off, the pose from the odometry should be reset at the end of the autonomous period to whatever our final pose is.
         * Second, the WPILib coordinate system uses North, West Up system as their positives. See initialization of inverter for why I hate it so much.
         * Third, 0 degrees/0 radians on the unit circle is at the front of the robot and 90 degrees is at the west position. However, East is -90 degrees rather than 270.
         * Fourth, the origin is always at the blue side of the field, all the way to the right when standing behind the driver stations. 
         * Fifth, the center of the reef:
         * 176.75 from driverstation wall to center along x. 144 inches from driverstation to edge of reef, reef if 65.5 inches wide excluding tape, divide by 2 and add 144 to get 176.75. must be converted to meters.
         * it's 109.13 inches along the y axis from the edge of the driverstation, but this is not taking into account the coral loading area thing.
         * 158.5 is the y axis, i used the apriltag map and matched y coords of tags 18 and 21. confirmed x value at 176.745
         */

        //from blue alliance origin, may need to convert for red alliance. 
        final double reefCenterXBlue = 4.48945; //in meters, 176.75 inches
        final double reefCenterYBlue = 4.0259; //in meters, 158.5 inches.

        double xDistanceToCenter = reefCenterXBlue - currentPose.getX(); //where we need to be vs where we are
        double yDistanceToCenter = reefCenterYBlue - currentPose.getY();

        Translation2d coordinatesToCenterOfReef = new Translation2d(xDistanceToCenter, yDistanceToCenter);

        return  coordinatesToCenterOfReef;
    }
    
    /**
     * Drives with swerve during the autonomous period
     * @param chassisSpeeds
     */
    public void driveAutonomous (ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        m_frontRight.setDesiredState(moduleStates[0]);
        m_frontLeft.setDesiredState(moduleStates[1]);
        m_backLeft.setDesiredState(moduleStates[2]);
        m_backRight.setDesiredState(moduleStates[3]);
        
    }
    /**
     * Updates the position of the robot relative to where its starting position
     */
    public void updateOdometry() { //it may have to be in the right order
        positions[0] = new SwerveModulePosition(m_frontRight.getDifferentState().speedMetersPerSecond, m_frontRight.getState().angle);
        positions[1] = new SwerveModulePosition(m_frontLeft.getDifferentState().speedMetersPerSecond, m_frontLeft.getState().angle);
        positions[2] = new SwerveModulePosition(m_backLeft.getDifferentState().speedMetersPerSecond, m_backLeft.getState().angle);
        positions[3] = new SwerveModulePosition(m_backRight.getDifferentState().speedMetersPerSecond, m_backRight.getState().angle);

        Pose2d m_pose = m_odometry.update(navx.getRotation2d(), positions);
    }

    /**
     * Gives the current position and rotation of the robot (meters) based on the wheel odometry from where the robot started
     * @return Pose2d of current robot position
     */
    public Pose2d getCurrentPose2d() {
        return m_odometry.getPoseMeters();
        
    }

    /**
     * Converts raw module states into chassis speeds 
     * @return chassis speeds object
     */
    public ChassisSpeeds getChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(m_backLeft.getState(), m_frontLeft.getState(), m_backRight.getState(), m_frontRight.getState());
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     * @param defenseHoldingMode Whether we invert our wheels to prevent slide during defense.
     * @param reefRotateCorresponder int between 0 and 2, where 0 is do nothing, 1 is rotate clockwise, 2 is rotate counterclockwise
     */
     @SuppressWarnings("ParameterName")
     public void drive(double driverXStick, double driverYStick, double driverRotateStick, boolean fieldRelative, boolean defenseHoldingMode, int reefRotateCorresponder, boolean onBlueAlliance) {
         double offset = (navx.getAngle());
         double offsetRadians = Math.toRadians(offset);
         Rotation2d robotRotation = new Rotation2d(offsetRadians); 
         
         //reefRotater setup. Basically, we get our robot pose from whatever way, then we figure out if we are doing rotator or not. If no, we set our center of rotation to the center of the robot, and if yes, we set our center of rotation to the center of the reef. 
         //then, we are able to automatically rotate around it at a fixed radius, which we can look into changing using the triggers later if we really care. 
         if (reefRotateCorresponder == 0) {
             var swerveModuleStates = m_kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(driverXStick * invert, driverYStick* invert, driverRotateStick* invert, robotRotation));
 
             SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
 
             m_frontRight.setDesiredState(swerveModuleStates[0]);
             m_frontLeft.setDesiredState(swerveModuleStates[1]);
             m_backLeft.setDesiredState(swerveModuleStates[2]);//NOTE FIX THESE TWO BACK ONES LATER WE MAY NEED THESE IN RIGHT ORDER IN AUTO.
             m_backRight.setDesiredState(swerveModuleStates[3]);
         } else if(reefRotateCorresponder == 1) {
 
             var swerveModuleStates = m_kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, .5, robotRotation), getPoseToReefCenter(getCurrentPose2d(), onBlueAlliance));
 
             SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
 
             m_frontRight.setDesiredState(swerveModuleStates[0]);
             m_frontLeft.setDesiredState(swerveModuleStates[1]);
             m_backLeft.setDesiredState(swerveModuleStates[2]);
             m_backRight.setDesiredState(swerveModuleStates[3]);
         } else if (reefRotateCorresponder == 2) {
             var swerveModuleStates = m_kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, -.5, robotRotation), getPoseToReefCenter(getCurrentPose2d(), onBlueAlliance));
 
             SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
 
             m_frontRight.setDesiredState(swerveModuleStates[0]);
             m_frontLeft.setDesiredState(swerveModuleStates[1]);
             m_backLeft.setDesiredState(swerveModuleStates[2]);
             m_backRight.setDesiredState(swerveModuleStates[3]);
         }
          
     }

}