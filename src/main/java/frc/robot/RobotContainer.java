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
import frc.robot.Constants.DriveConst;
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
    private Timer m_timer = new Timer();

    /*
     * Collection of driver status buttons and joysticks, initially set to do nothing. 
     */
    private double driverXStick = 0;
    private double driverYStick = 0;
    private double driverRotStick = 0;
    private boolean manualOperateElevator = true;
    private boolean isAutoAligning = false;
    private boolean isAutoAutoAligning = true; //auto align in autonomous, just a switch so we can run something once. 
    //private boolean areWeGood = false;

    //konami code: up up down down left right left right B A 
    XboxController driverController = new XboxController(0);
    XboxController operatorController = new XboxController(1);

    public RobotContainer () {

    }

    public void AutoStart() {
        m_timer.reset();
        m_timer.start();
    }

    public void Auto() { //TODO WILL NEED TO PUT IN FUNCTION TO RAISE ELEVATOR HEIGHT TALL ENOUGH SO OUR LIMELIGHT CAN SEE
        if(isAutoAutoAligning) {
            isAutoAutoAligning = false;
            autoAligner.AutoAlignStart(true);
        }
        autoAligner.AutoAlignPeriodic();

        if(m_timer.get() > 7) {
            m_Elevator.setPosition(ElevatorConst.L4state);
        } 
        if (m_timer.get() > 13) {
            m_Cradle.driveMotorNoPID(1, false);
        }
    }

    public void drive() {
        driverXStick = driverController.getRawAxis(0);
        driverYStick = driverController.getRawAxis(1);
        driverRotStick = driverController.getRawAxis(4);

        driverXStick *= DriveConst.speedLimiter;
        driverYStick *= DriveConst.speedLimiter;
        driverRotStick *= .5;


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
            m_Elevator.driveMotorNoPID((operatorController.getRawAxis(1)), false);
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

        //axis 2 is ____, axis 3 is ____ 
        if(operatorController.getRawAxis(2) > .2) {
            m_Cradle.driveMotorNoPID(Math.pow(operatorController.getRawAxis(2), 3), true);
        } else if(operatorController.getRawAxis(3) > .2) {
            m_Cradle.driveMotorNoPID(Math.pow(operatorController.getRawAxis(3), 3), false);
        } else {
            m_Cradle.driveMotorNoPID(0, false);
        }

        if(operatorController.getRawButton(10)) {
            m_Elevator.resetEncoders();
        }
    }

    public void testElevator() {
        m_Elevator.driveMotorNoPID(operatorController.getRawAxis(1), false);
    }

    // public boolean areWeGood() {
    //     if(!areWeGood) {
    //         areWeGood = true; //we're so back
    //     } else {
    //         areWeGood = false; //it's so over
    //     }
    //     return areWeGood;
    // }
}