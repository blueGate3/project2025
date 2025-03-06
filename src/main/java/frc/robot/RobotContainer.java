package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.security.cert.TrustAnchor;

import javax.naming.OperationNotSupportedException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Cradle;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();
    private final Elevator m_Elevator = new Elevator();
    private final SendableChooser<Command> autoChooser;
    private Timer mTimer = new Timer();
    /*
     * Collection of driver status buttons and joysticks, initially set to do nothing. 
     */
    private double driverXStick = 0;
    private double driverYStick = 0;
    private double driverRotStick = 0;
    private boolean driverXButton = false;
    private boolean driverYButton = false;
    private boolean driverAButton = false;
    private boolean driverBButton = false;
    private boolean reefRotate = false;
    private double driverLeftTrigger = 0;
    private double driverRightTrigger = 0;
    
    // private SlewRateLimiter xDriveLimiter = new SlewRateLimiter(5);
    // private SlewRateLimiter yDriveLimiter = new SlewRateLimiter(5);
    // private SlewRateLimiter rotDriveLimiter = new SlewRateLimiter(5);

    private double positionRotations;
    
    private boolean operatorXButton = false;
    private boolean operatorYButton = false;
    private boolean operatorAButton = false;
    private boolean operatorBButton = false;
    private double operatorLeftTrigger = 0;
    private double operatorRightTrigger = 0;
    private boolean operatorLB = false;
    private boolean operatorRB = false;

    private boolean phaseOne = false;
    private boolean phaseTwo = false;
    private boolean phaseThree = false;

    private boolean manualOperateElevator = false;

    //konami code: up up down down left right left right B A 

    XboxController driverController = new XboxController(0);
    XboxController operatorController = new XboxController(1);

    public RobotContainer () {
        autoChooser = AutoBuilder.buildAutoChooser();
        //autoChooser.addOption("TestPath-Rotate", x);
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        try{
            // Load the path you want to follow using its name in the GUI
            PathPlannerPath path = PathPlannerPath.fromPathFile("RedPathOne");
    
            // Create a path following command using AutoBuilder. This will also trigger event markers.
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
      }

    //for fun and to look really cool when first connected to FMS
    public void setupDataSpew() {
        if (DriverStation.isFMSAttached()) {
            System.out.println("Connected to the Field Management System.");
            System.out.println("Current match number: " + DriverStation.getMatchNumber());
            System.out.println("Current Alliance: " + DriverStation.getAlliance());
            System.out.println("Current DriverStation Location: " + DriverStation.getLocation());
            System.out.println("Current Event: " + DriverStation.getEventName());
            System.out.println("Game specific message: " + DriverStation.getGameSpecificMessage());
            System.out.println("Good luck!!!");
        }
    }

    public void readDriverController() {
        //Driver stick getters
        

        // driverXStick = xDriveLimiter.calculate(driverController.getRawAxis(0));
        // driverYStick = yDriveLimiter.calculate(driverController.getRawAxis(1));
        // driverRotStick = -rotDriveLimiter.calculate(driverController.getRawAxis(2));

        driverXStick = driverController.getRawAxis(0);
        driverYStick = driverController.getRawAxis(1);
        driverRotStick = driverController.getRawAxis(4);

        driverLeftTrigger = driverController.getLeftTriggerAxis();
        driverRightTrigger = driverController.getRightTriggerAxis();
        driverXButton = driverController.getXButton(); //x is hold
        driverYButton = driverController.getYButton(); //y is hold
        driverAButton = driverController.getAButton(); //a is hold
        driverBButton = driverController.getBButton(); //b is hold


        //rumbles controller so driver knows the match has started and when endgame has started.
        //if( DriverStation.getMatchTime() == 15 || DriverStation.isTeleop()) {

        //}

        /*
         * Example ways to get different types of input:
         * driverXButton = driverController.getXButton(); //This checks every 20ms if it's pressed, you would need to hold the button
         * driverXButton = driverController.getXButtonPressed(); //whether button was pressed since last check, this might be good for toggling on commands.
         * driverXButton = driverController.getRawButtonReleased(0); //true if button went from held down to not pressed since last check.
         */
    }

    public void readOperatorController() {
        operatorAButton = operatorController.getAButton();
        operatorBButton = operatorController.getBButton();
        operatorXButton = operatorController.getXButton();
        operatorYButton = operatorController.getYButton();

        operatorLeftTrigger = operatorController.getLeftTriggerAxis();
        operatorRightTrigger = operatorController.getRightTriggerAxis();
        operatorLB = operatorController.getLeftBumperButton();
        operatorRB = operatorController.getRightBumperButton();
    }

    /**
     * Gordon Ramsey himself couldn't do better. 
     */
    public void letDriverCook () {



        if(driverXButton) {
            //drive slowly
            drivetrain.drive(driverXStick/10, driverYStick/10, (driverRotStick + .01)/10, true, false, false);
        } else if (driverAButton) {
            //driveRobotRelative
        } else if (driverYButton) {
            //drive slowly and robot relative
            drivetrain.drive(driverXStick/10, driverYStick/10, (driverRotStick + .01)/10, false, false, false);
        } else if(driverBButton) {
            //creates X with wheels so we can't be pushed around.
            drivetrain.drive(0, 0, 0, true, false, true);
        } else {
            drivetrain.drive(driverXStick, driverYStick, driverRotStick + .01, true, false, false); //the rotation being .01% is so we have a holding position in the rotate position, so the wheels are all good, but there's not enough power to actually drive it. 
        }
    }

    public void letOperatorCook() {
        //determines/switches mode
        if(operatorController.getRawButton(2)) {
            manualOperateElevator = false;
        }
        if(operatorController.getRawButton(5)) { //left bumper
            manualOperateElevator = true;
        }
        //Main logic
        if(manualOperateElevator) {

            m_Elevator.driveMotorNoPID(Math.pow((operatorController.getRawAxis(1)),3), true);

        } else {
            if(operatorYButton) {
                positionRotations = 32;
                System.out.println("Y");
            } else if (operatorBButton) {
                System.out.println("B");
                positionRotations = 48;
            } else if (operatorXButton) {
                System.out.println("X");
                positionRotations = 70;
            } //else if (operatorRB) {
            //     positionRotations = 40;
            // }

            m_Elevator.driveMotor(positionRotations);

        }
        //m_Elevator.driveMotorNoPID(operatorController.getRawAxis(1), true);
    }

    public void manualAuto(double driveRots, double turnRots) {
        drivetrain.driveManualAuto(driveRots, turnRots);
    }

    public void resetDriveEncoder() {
        drivetrain.resetDriveEncoders();
    }

    public void autopath() {
        manualAuto(10, 0);
        if(mTimer.get() > 2) {
            manualAuto(30, 1);
        }   if(mTimer.get() > 5) {
            manualAuto(20, .5);
            mTimer.stop();
            resetTimer();
        }
    }

    public void startTimer() {
        mTimer.start();
    }
    public void resetTimer() {
        mTimer.reset();
    }

}