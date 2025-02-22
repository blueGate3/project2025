package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
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
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.DriveCommands.ReefRotateCommand;
import frc.robot.Subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();
    //public final SmartDashboardUpdater smartDashboardUpdater = new SmartDashboardUpdater();
    public Trigger reefRotateTrigger;
    public Trigger robotRelativeTrigger;
    public Trigger driveSlowTrigger;
    public Trigger driveRawTrigger;
    public Trigger driveRegularLinear;

    CommandXboxController driverController = new CommandXboxController(0); // 0 is the USB Port to be used as indicated on the Driver Station
    CommandXboxController operatorController = new CommandXboxController(1);

    public RobotContainer () {
        //setDefaultCommands();
        configureButtonBindings();
        //setDefaultCommands();
        
    }
    


    public void configureButtonBindings () {

    }

    public void setDefaultCommands () {
        drivetrain.setDefaultCommand(
                drivetrain.driveRegularCommand(
                // Math.pow(driverController.getRawAxis(0), 3),
                // Math.pow(driverController.getRawAxis(1), 3),
                // Math.pow(driverController.getRawAxis(2), 3)
                1, 0, 0
        ));
    }

    // public void driverShuffleBoardUpdater() {
    //     SmartDashboard.putBoolean("ReefRotate Command Running", reefRotateTrigger.getAsBoolean());
    //     SmartDashboard.putBoolean("RobotRelative Command Running", robotRelativeTrigger.getAsBoolean());
    //     SmartDashboard.putBoolean("FineTuneDrive Command Running", driveSlowTrigger.getAsBoolean());
    //     SmartDashboard.putBoolean("DriveRaw Command Running", driveRawTrigger.getAsBoolean());
    //     SmartDashboard.putBoolean("DriveRegular (Linear) Command Running", driveRegularLinear.getAsBoolean());
    // }

    public void configureDriverCommands() {
        //ReefRotatorCommand, goes with whichever trigger has a higher value greater than .2
        reefRotateTrigger = new Trigger(driverController.rightTrigger(.2))
        .or(driverController.leftTrigger(.2))
        .onTrue(drivetrain.ReefRotateCommand(
            driverController.getLeftTriggerAxis(), 
            driverController.getRightTriggerAxis()
            ));

        //drive linear, robot relative for camera fine tune alignment
        robotRelativeTrigger =new Trigger(driverController.leftStick())
        .toggleOnTrue(drivetrain.driveRobotRelativeCommand(
            driverController.getRawAxis(0), 
            driverController.getRawAxis(1), 
            driverController.getRawAxis(2)
            ));

        //drive slow, linear, robot relative for camera fine tune alignment
        driveSlowTrigger = new Trigger(driverController.b())
        .toggleOnTrue(drivetrain.driveSlowCommand(
            driverController.getRawAxis(0), 
            driverController.getRawAxis(1), 
            driverController.getRawAxis(2),
            50
            ));

        //no cosine compensation, slew rates, etc. just raw driving
        driveRawTrigger = new Trigger(driverController.a())
        .toggleOnTrue(drivetrain.driveRawCommand(
            driverController.getRawAxis(0), 
            driverController.getRawAxis(1), 
            driverController.getRawAxis(2)
            ));

        //drives without x^3, linear input
        driveRegularLinear = new Trigger(driverController.x())
        .toggleOnTrue(drivetrain.driveRegularCommand(
            driverController.getRawAxis(0), 
            driverController.getRawAxis(1), 
            driverController.getRawAxis(2)
            ));
        
        
        //

        // driverController.getAButton().onTrue(drivetrain.driveRobotRelativeCommand(
        //     driverController.getRawAxis(0), 
        //     driverController.getRawAxis(1), 
        //     driverController.getRawAxis(2)));
        
    }

    /**
     * Okay. Here we go. Focus. Speed. I am speed. One winner. 42 losers. I eat losers for breakfast.
     * Breakfast? Wait. maybe I should have had breakfast. A little brecky could be good for me. 
     * No, non no. Stay focused. Speed! I'm faster than fast, quicker than quick! I'm LIGHTINING!!
     */
    public void letDriverCook() {
        double driverXStick = -driverController.getRawAxis(0);
        double driverYStick = -driverController.getRawAxis(1);
        double driverRotateStick = driverController.getRawAxis(2);
        double driverRightTrigger = driverController.getRightTriggerAxis();
        double driverLeftTrigger = driverController.getLeftTriggerAxis();

        boolean reefRotate = false;

        //if either triggers pressed more then .25 (that's a crude deadband), rotate with that trigger value. Right is positive, left is negative. 
        // if ((driverRightTrigger >.25)) { //if either of the triggers have been pressed sufficiently (.05 in as crude deadband)
        //     reefRotate = true;
        //     drivetrain.drive(0, 0, driverRightTrigger, true, true);
        // } else if (driverLeftTrigger >.25) {
        //     reefRotate = true;
        //     drivetrain.drive(0, 0, -driverLeftTrigger, true, true);
        // } else { //drives regularly.
            drivetrain.drive(driverXStick, driverYStick, driverRotateStick, true, false); //negative y value coz its backwards
        //}

        // SmartDashboard.putNumber("Driver X Stick", driverXStick);
        // SmartDashboard.putNumber("Driver X Stick", driverXStick);
        // SmartDashboard.putNumber("Driver X Stick", driverXStick);
        // SmartDashboard.putBoolean("ReefRotate Mode", reefRotate);

    }
}