package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.*;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Elevator;
import frc.robot.Constants.DriveConst;
import frc.robot.Constants.ElevatorConst;
import frc.robot.Subsystems.AutoAlign;
import frc.robot.Subsystems.Cradle;

public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();
    private final Cradle m_Cradle = new Cradle();
    private final Elevator m_Elevator = new Elevator();
    private AutoAlign m_AutoAlign = new AutoAlign(drivetrain);
    private TrapezoidProfile.State m_wantedState = ElevatorConst.homeState;
    private Timer m_timer = new Timer();

    /*
     * Collection of driver status buttons and joysticks, initially set to do nothing. 
     */
    private double driverXStick = 0;
    private double driverYStick = 0;
    private double driverRotStick = 0;
    private boolean manualOperateElevator = true;
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

    // public void Auto() { //TODO WILL NEED TO PUT IN FUNCTION TO RAISE ELEVATOR HEIGHT TALL ENOUGH SO OUR LIMELIGHT CAN SEE
    //     if(isAutoAutoAligning) {
    //         isAutoAutoAligning = false;
    //         autoAligner.AutoAlignStart(true);
    //     }
    //     autoAligner.AutoAlignPeriodic();

    //     if(m_timer.get() > 7) {
    //         m_Elevator.setPosition(ElevatorConst.L4state);
    //     } 
    //     if (m_timer.get() > 13) {
    //         m_Cradle.driveMotorNoPID(1, false);
    //     }
    // }

    public void drive() {
        driverXStick = driverController.getRawAxis(0);
        driverYStick = driverController.getRawAxis(1);
        driverRotStick = driverController.getRawAxis(4);

        driverXStick *= DriveConst.speedLimiter;
        driverYStick *= DriveConst.speedLimiter;
        driverRotStick *= .5;

        if(driverController.getLeftBumperButton() || driverController.getRightBumperButton()) {
            if(driverController.getLeftBumperButton()) {
                m_AutoAlign.autoAlignReef(true);
            } else if (driverController.getRightBumperButton()) {
                m_AutoAlign.autoAlignReef(false);
            }
        } else {
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