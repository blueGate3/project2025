package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.*;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.LimelightHelpers;
import frc.robot.Subsystems.NewAutoAlign;
import frc.robot.Constants.DriveConst;
import frc.robot.Constants.ElevatorConst;
import frc.robot.Subsystems.AutoAlign;
import frc.robot.Subsystems.Cradle;

public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();
    private final Cradle m_Cradle = new Cradle();
    private final Elevator m_Elevator = new Elevator();
    private final NewAutoAlign m_NewAutoAlign = new NewAutoAlign(drivetrain);
    private AutoAlign m_AutoAlign = new AutoAlign(drivetrain);
    private TrapezoidProfile.State m_wantedState = ElevatorConst.homeState;
    private Timer m_timer = new Timer();

    /*
     * Collection of driver status buttons and joysticks, initially set to do nothing. 
     */
    private double driverXStick = 0;
    private double driverYStick = 0;
    private double driverRotStick = 0;
    private boolean manualOperateElevator = false;
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

    public void Auto() { 
        if(m_timer.get() < 2) {
            m_Elevator.setPosition(ElevatorConst.L2state);
        } 
        if(m_timer.get() > 2.1 && m_timer.get() < 6) {
            drivetrain.drive(0, .05, 0, false, false);
        }
        if(m_timer.get() > 6.1 && m_timer.get() < 11) {
            m_NewAutoAlign.aimX(false);
        } 
        if(m_timer.get() > 11.1 && m_timer.get() < 14) {
            m_Elevator.setPosition(ElevatorConst.L4state);
        }
        if(m_timer.get() > 14) {
            m_Cradle.driveMotorNoPID(.7, false);
        }

    }

    public void drive() {
        driverXStick = driverController.getRawAxis(0);
        driverYStick = driverController.getRawAxis(1);
        driverRotStick = driverController.getRawAxis(4);
        
        driverXStick = MathUtil.applyDeadband(driverXStick, .08);
        driverYStick = MathUtil.applyDeadband(driverYStick, .08);
        driverRotStick = MathUtil.applyDeadband(driverRotStick, .08);
        if(driverController.getLeftTriggerAxis() > .4 || driverController.getRightTriggerAxis() > .4) { //slowmode enabled
            driverXStick *= .15;
            driverYStick *= .15;
            driverRotStick *= .3;
        } else {
            driverXStick *= DriveConst.speedLimiter;
            driverYStick *= DriveConst.speedLimiter;
            driverRotStick *= .4;
        }

        if(driverController.getYButton()) {
            drivetrain.resetNavX();
        }

        if(driverController.getLeftBumperButton() || driverController.getRightBumperButton()) {
            if(driverController.getLeftBumperButton()) {
                m_NewAutoAlign.align(true);
            } else if (driverController.getRightBumperButton()) {
                m_NewAutoAlign.align(false);
            }
        } else if(driverController.getAButton()) {
            m_NewAutoAlign.approach();
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
            m_Elevator.driveMotorNoPID((MathUtil.applyDeadband(operatorController.getRawAxis(1), .1)), false);
        } else {

            if(operatorController.getYButton()) {
                m_wantedState = ElevatorConst.L4state;
            }
            if(operatorController.getBButton()) {
                m_wantedState = ElevatorConst.L2state;
            }
            if(operatorController.getXButton()) {
                m_wantedState = ElevatorConst.L3state;
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

    public void runLimelightLED() {
        if(driverController.getRawButton(2)) {
            LimelightHelpers.setLEDMode_ForceOn("");
        } else {
            LimelightHelpers.setLEDMode_ForceOff("");
        }
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