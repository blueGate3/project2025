package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.DriveCommands.ReefRotateCommand;
import frc.robot.Subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();
    //public final SmartDashboardUpdater smartDashboardUpdater = new SmartDashboardUpdater();

    XboxController driverController = new XboxController(0); // 0 is the USB Port to be used as indicated on the Driver Station
    XboxController operatorController = new XboxController(1);

    public RobotContainer () {
        setDefaultCommands();
        configureButtonBindings();

    }
    


    public void configureButtonBindings () {

    }

    public void setDefaultCommands () {
        drivetrain.setDefaultCommand(
            drivetrain.driveRegularlyCommand(
                driverController.getRawAxis(0), 
                driverController.getRawAxis(1), 
                driverController.getRawAxis(2))
        );
    }

    public void driverCommands() {
    //     new Trigger(driverController.getAButton()).or
    //     .whileTrue(drivetrain.ReefRotateCommand(driverController.getRawAxis(2)));
        
        //ReefRotatorCommand, goes with whichever trigger has a higher value greater than .2
        new Trigger(drivetrain.eitherTriggerPressed(driverController.getLeftTriggerAxis(), driverController.getRightTriggerAxis(), .2))
        .onTrue(drivetrain.ReefRotateCommand(driverController.getLeftTriggerAxis(), driverController.getRightTriggerAxis()));
        
        //TODO put in a drive slow method for fine tunement (like divide all speeds by 50 or smth), add in x^3 robot control, and make a field relative button. should be a fun night. 
    }

    /**
     * Okay. Here we go. Focus. Speed. I am speed. One winner. 42 losers. I eat losers for breakfast.
     * Breakfast? Wait. maybe I should have had breakfast. A little brecky could be good for me. 
     * No, non no. Stay focused. Speed! I'm faster than fast, quicker than quick! I'm LIGHTINING!!
     */
    public void letDriverCook() {
        double driverXStick = driverController.getRawAxis(0);
        double driverYStick = -driverController.getRawAxis(1);
        double driverRotateStick = driverController.getRawAxis(2);
        double driverRightTrigger = driverController.getRightTriggerAxis();
        double driverLeftTrigger = driverController.getLeftTriggerAxis();

        boolean reefRotate = false;

        //if either triggers pressed more then .05 (that's a crude deadband), rotate with that trigger value. Right is positive, left is negative. 
        if ((driverRightTrigger >.25)) { //if either of the triggers have been pressed sufficiently (.05 in as crude deadband)
            reefRotate = true;
            drivetrain.drive(0, 0, driverRightTrigger, true, true);
        } else if (driverLeftTrigger >.25) {
            reefRotate = true;
            drivetrain.drive(0, 0, -driverLeftTrigger, true, true);
        } else { //drives regularly.
            drivetrain.drive(driverXStick, driverYStick, driverRotateStick, true, false); //negative y value coz its backwards
        }

        SmartDashboard.putNumber("Driver X Stick", driverXStick);
        SmartDashboard.putNumber("Driver X Stick", driverXStick);
        SmartDashboard.putNumber("Driver X Stick", driverXStick);
        SmartDashboard.putBoolean("ReefRotate Mode", reefRotate);

    }
}