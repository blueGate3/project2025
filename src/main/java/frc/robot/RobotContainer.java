package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.security.cert.TrustAnchor;

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
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();
    private final SendableChooser<Command> autoChooser;
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
    
    private SlewRateLimiter xDriveLimiter = new SlewRateLimiter(5);
    private SlewRateLimiter yDriveLimiter = new SlewRateLimiter(5);
    private SlewRateLimiter rotDriveLimiter = new SlewRateLimiter(5);
    
    

    //konami code: up up down down left right left right B A 

    CommandXboxController driverCommandController = new CommandXboxController(0); // 0 is the USB Port to be used as indicated on the Driver Station
    CommandXboxController operatorCommandController = new CommandXboxController(1);

    XboxController driverController = new XboxController(0);
    XboxController operatorController = new XboxController(1);

    public RobotContainer () {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
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
        driverXStick = xDriveLimiter.calculate(driverController.getRawAxis(0));
        driverYStick = yDriveLimiter.calculate(driverController.getRawAxis(1));
        driverRotStick = rotDriveLimiter.calculate(driverController.getRawAxis(2));
        driverLeftTrigger = driverController.getLeftTriggerAxis();
        driverRightTrigger = driverController.getRightTriggerAxis();
        driverXButton = driverController.getXButton(); //x is hold
        driverYButton = driverController.getYButton(); //y is hold
        driverAButton = driverController.getAButton(); //a is hold
        driverBButton = driverController.getBButton(); //b is hold


        if(driverController.getLeftTriggerAxis() > .25 || driverController.getRightTriggerAxis() > .25) {
            reefRotate = true; //will run reefRotate with desired trigger values. 
        } else {
            reefRotate = false; //just resets to false.
        }

        //TODO figure out hold down buttons

        //rumbles controller so driver knows the match has started and when endgame has started.
        if( DriverStation.getMatchTime() == 15 || DriverStation.isTeleop()) {

            driverController.setRumble(RumbleType.kBothRumble, .5);
        }

        /*
         * Example ways to get different types of input:
         * driverXButton = driverController.getXButton(); //This checks every 20ms if it's pressed, you would need to hold the button
         * driverXButton = driverController.getXButtonPressed(); //whether button was pressed since last check, this might be good for toggling on commands.
         * driverXButton = driverController.getRawButtonReleased(0); //true if button went from held down to not pressed since last check.
         */
    }
    /**
     * Gordon Ramsey himself couldn't do better. 
     */
    public void letDriverCook () {
        if(driverXButton) {
            //drive slowly
            drivetrain.drive(driverXStick/20, driverYStick/20, (driverRotStick + .0001)/20, true, false, false);
        } else if (driverYButton) {
            //driveRobotRelative
            drivetrain.drive(driverXStick, driverYStick, driverRotStick + .0001, false, false, false);
        } else if (driverAButton) {
            //drive slowly and robot relative
            drivetrain.drive(driverXStick/20, driverYStick/20, (driverRotStick + .0001)/20, false, false, false);
        } else if(driverBButton) {
            //creates X with wheels so we can't be pushed around.
            drivetrain.drive(0, 0, 0, false, false, true);
        } else {
            drivetrain.drive(driverXStick, driverYStick, driverRotStick + .0001, true, false, false); //the rotation being .01% is so we have a holding position in the rotate position, so the wheels are all good, but there's not enough power to actually drive it. 
        }
    }

    public void driveAutoManual(double xSpeed, double ySpeed, double rot, long time) throws InterruptedException {
        drivetrain.drive(xSpeed, ySpeed, rot, true, false, false);
    }
    
    
}