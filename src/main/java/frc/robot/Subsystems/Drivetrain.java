// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.thethriftybot.Conversion;

import java.io.IOException;
import java.util.function.BooleanSupplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
//import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
import frc.robot.Robot;

/** Represents a swerve drive style drivetrain. */

public class Drivetrain extends SubsystemBase {
    // kMaxSpeed was 2 AND kmaxangularspeed was pi/3 (before testing [district champs])
    // SOLID SPEEDS 3.25 M/S /AND PI/2.25 ROT/S
    public static final double kMaxSpeed = 5.88; // 5.88 meters per second or 19.3 ft/s (max speed of SDS Mk4i with Vortex motor)
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
    //boolean onBlueAlliance;

    int invert = 1; //this will change depending on the alliance we are put on, it will be multiplied by -1 if we are red alliance and then multiplied by all of the drive inputs so we still drive the correct way and can remain blue alliance oriented for apriltags. 
    //more information can be found at https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html 

    Pose2d m_pose;
    private final AHRS navx = new AHRS(NavXComType.kMXP_SPI); 

    // Locations of each swerve module relative to the center of the robot
    private final Translation2d m_frontRightLocation = new Translation2d( 0.3175, -0.3175);//side length total is at 29.5 inches including modules. Divided by 2 and set to meters is .37465 meters from one side to the tip of the module
    private final Translation2d m_frontLeftLocation = new Translation2d(0.3175,  0.3175);//the frc kinematics section has the coordinates so x is front-back, where front is positive, and y is left-right, where left is positive. it's communist to the extreme but will affect the way we initialize our module locations.
    private final Translation2d m_backLeftLocation = new Translation2d(-0.3175,  0.3175);//continued: that's the reason for the strange abnormal abhorrent disgusting affronts-before-God translation signs. 
    private final Translation2d m_backRightLocation = new Translation2d( -0.3175, -0.3175);

    // Constructor for each swerve module
    private final SwerveModule m_frontRight = new SwerveModule(1, 2, false, false); //
    private final SwerveModule m_frontLeft = new SwerveModule(3, 4, false, false); //
    private final SwerveModule m_backLeft = new SwerveModule(5, 6, false, false); //
    private final SwerveModule m_backRight = new SwerveModule(7, 8, false, false); //

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
    public RobotConfig config;

    // Constructor
    public Drivetrain() {
        m_initialStates = new SwerveDriveKinematics(m_frontRightLocation, m_frontLeftLocation, m_backLeftLocation, m_backRightLocation);
        navx.reset();
        navx.setAngleAdjustment(90);
        m_odometry = new SwerveDriveOdometry(
            m_kinematics, 
            navx.getRotation2d(), initialPositions
        );
        
        try {
            config = RobotConfig.fromGUISettings();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        AutoBuilder.configure(
            this::getCurrentPose2d, // Robot pose supplier
            this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveAutonomous(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }
    /**
     * Updates the odometry pose
     * @param pose Current pose of the robot. TODO allow for correction using apriltag to set pose. 
     */
    public void setPose (Pose2d pose) {
        m_odometry.resetPosition(new Rotation2d(pose.getRotation().getRadians()), initialPositions, pose);
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
    public void updateOdometry() {
        positions[0] = new SwerveModulePosition(m_frontRight.getDifferentState().speedMetersPerSecond, m_frontRight.getState().angle);
        positions[1] = new SwerveModulePosition(m_frontLeft.getDifferentState().speedMetersPerSecond, m_frontLeft.getState().angle);
        positions[2] = new SwerveModulePosition(m_backLeft.getDifferentState().speedMetersPerSecond, m_backLeft.getState().angle);
        positions[3] = new SwerveModulePosition(m_backRight.getDifferentState().speedMetersPerSecond, m_backRight.getState().angle);
        m_pose = m_odometry.update(navx.getRotation2d(), positions);
    }

    /**
     * Gives the current position and rotation of the robot (meters) based on the wheel odometry from where the robot started
     * @return Pose2d of current robot position
     */
    public Pose2d getCurrentPose2d() {
        m_pose = m_odometry.getPoseMeters();
        return m_pose;
    }

    /**
     * Converts raw module states into chassis speeds 
     * @return chassis speeds object
     */
    public ChassisSpeeds getChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(m_frontRight.getState(), m_frontLeft.getState(), m_backLeft.getState(), m_backRight.getState());
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     * @param reefRotate whether we are rotating reef
     */
     @SuppressWarnings("ParameterName")
     public void drive(double driverXStick, double driverYStick, double driverRotateStick, boolean fieldRelative, boolean reefRotate, boolean defenseHoldingMode) {
        Rotation2d robotRotation = new Rotation2d(Math.toRadians(navx.getAngle()));
        double xFinal = driverXStick;
        double yFinal = driverYStick;
        double rotFinal = driverRotateStick;


        var swerveModuleStates = m_kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xFinal, yFinal, rotFinal, robotRotation));
         //reefRotater setup. Basically, we get our robot pose from whatever way, then we figure out if we are doing rotator or not. If no, we set our center of rotation to the center of the robot, and if yes, we set our center of rotation to the center of the reef. 
         //then, we are able to automatically rotate around it at a fixed radius, which we can look into changing using the triggers later if we really care. 
        if(reefRotate) {
            swerveModuleStates = m_kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, driverRotateStick, robotRotation), getPoseToReefCenter(getCurrentPose2d(), true));
        }

        if(!defenseHoldingMode) {
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
            m_frontRight.setDesiredState(swerveModuleStates[0]);
            m_frontLeft.setDesiredState(swerveModuleStates[1]);
            m_backLeft.setDesiredState(swerveModuleStates[2]);
            m_backRight.setDesiredState(swerveModuleStates[3]);
        } else {
            //creates X pattern with wheels so we cant be pushed around. 
            m_frontLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d(3 * (Math.PI / 4))));
            m_frontRight.setDesiredState(new SwerveModuleState(0, new Rotation2d((Math.PI / 4))));
            m_backLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d((Math.PI / 4))));
            m_backRight.setDesiredState(new SwerveModuleState(0, new Rotation2d(3* (Math.PI / 4))));
        }
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

    // if(LeftTrigger > RightTrigger){
    //     return this.run(() -> drive(0, 0, LeftTrigger, true, true));
    // } else {
    //     return this.run(() -> drive(0, 0, RightTrigger, true, true));
    // }
}