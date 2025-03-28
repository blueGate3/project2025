package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Rangefinder;
import frc.robot.Constants.ElevatorConst;
import frc.robot.Subsystems.AlignToReefTagRelative;
import frc.robot.Subsystems.Cradle;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();
    private final Cradle m_Cradle = new Cradle();
    private final Elevator m_Elevator = new Elevator();
    private final Rangefinder rangefinderOne = new Rangefinder(3, 1);
    private final AlignToReefTagRelative autoAligner = new AlignToReefTagRelative(drivetrain); //please work
    private TrapezoidProfile.State m_wantedState = ElevatorConst.homeState;
    private Timer mTimer = new Timer();

    /*
     * Collection of driver status buttons and joysticks, initially set to do nothing. 
     */
    private double driverXStick = 0;
    private double driverYStick = 0;
    private double driverRotStick = 0;
    private boolean manualOperateElevator = false;
    private boolean isAutoAligning = false;

    //konami code: up up down down left right left right B A 
    XboxController driverController = new XboxController(0);
    XboxController operatorController = new XboxController(1);

    public RobotContainer () {

    }

    public void Auto() {
        
    }

    public void drive() {
        if(driverController.getLeftBumperButton() || driverController.getRightBumperButton()) { //get left bumper

            //called once initially during the holding loop so we start our auto alignment. 
            if(driverController.getLeftBumperButton() && !isAutoAligning) { //only activates the first time around so we aren't constantly doing a bunch of extra work. i should really switch to command based at some point. 
                autoAligner.AutoAlignStart(true);
                isAutoAligning = true;
            } else if(driverController.getRightBumperButton() && !isAutoAligning) { //only activates the first time around so we aren't constantly doing a bunch of extra work. i should really switch to command based at some point. 
                autoAligner.AutoAlignStart(false);
                isAutoAligning = true;
            }
            autoAligner.AutoAlignPeriodic(); //keeps running as long as a bumper is pressed. 
        } else { //rest of our main logic.
            isAutoAligning = false; //resets state back to false if the bumper stops being pressed so we can restart if need be. 

            drivetrain.drive(
            driverXStick, 
            driverYStick, 
            driverRotStick, 
            true,
            false);
        }
    }

    public void operate() {
        if(operatorController.getLeftBumperButton()) {
            manualOperateElevator = true;
        } else if ( operatorController.getRightBumperButton()) {
            manualOperateElevator = false;
        }


        if(manualOperateElevator) {
            m_Elevator.driveMotorNoPID(Math.pow(operatorController.getRawAxis(1), 3), true);
        } else {

            if(operatorController.getYButton()) {
                m_wantedState = ElevatorConst.L2state;
            }
            if(operatorController.getBButton()) {
                m_wantedState = ElevatorConst.L3state;
            }
            if(operatorController.getXButton()) {
                m_wantedState = ElevatorConst.L4state;
            }
            if(operatorController.getAButton()) {
                m_wantedState = ElevatorConst.homeState;
            }
            m_Elevator.setPosition(m_wantedState);

        }
    }
}