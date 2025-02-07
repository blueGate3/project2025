package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Subsystems.Drivetrain;

public class RobotContainer {
    public final Drivetrain drivetrain = new Drivetrain();
    //public final SmartDashboardUpdater smartDashboardUpdater = new SmartDashboardUpdater();

    XboxController driverController = new XboxController(0); // 0 is the USB Port to be used as indicated on the Driver Station
    XboxController operatorController = new XboxController(1);
    int reefRotate;

    public RobotContainer () {
        setDefaultCommands();
        configureButtonBindings();

    }
    


    public void configureButtonBindings () {
        if(driverController.getLeftBumperButtonPressed()){
            reefRotate=2; //left is rotate counterclockwise
        } else if (driverController.getRightBumperButtonPressed()) {
            reefRotate = 1;
        } else {
            reefRotate = 0;
        }


        drivetrain.drive(driverController.getLeftX(), driverController.getLeftY(), driverController.getRightX(), true, false, reefRotate);
        SmartDashboard.putNumber("X Axis", driverController.getLeftX());
        SmartDashboard.putNumber("Y Axis", driverController.getLeftY());
        SmartDashboard.putNumber("Rot Axis", driverController.getRightX());
        SmartDashboard.putBoolean("Field Relative", true);
        

    }

    public void setDefaultCommands () {
    
    }

}