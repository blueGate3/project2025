package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Cradle;

public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();
    private final Elevator m_Elevator = new Elevator();
    private final Cradle m_Cradle = new Cradle();
    private Timer mTimer = new Timer();
    /*
     * Collection of driver status buttons and joysticks, initially set to do nothing. 
     */
    private double driverXStick = 0;
    private double driverYStick = 0;
    private double driverRotStick = 0;

    private double positionRotations = 0;
    private final double elevatorOffset = 5.5; //inches, it' disitance from bottom of tray to ground.
    private boolean manualOperateElevator = false;

    //konami code: up up down down left right left right B A 
    XboxController driverController = new XboxController(0);
    XboxController operatorController = new XboxController(1);

    public RobotContainer () {

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
        driverXStick = Math.pow(driverController.getRawAxis(0), 3);
        driverYStick = Math.pow(driverController.getRawAxis(1), 3);
        driverRotStick = Math.pow(driverController.getRawAxis(4), 3);
    }

    /**
     * Gordon Ramsey himself couldn't do better. 
     */
    public void letDriverCook () {
        if(driverController.getRawButton(2)) {
            //creates X with wheels so we can't be pushed around.
            drivetrain.drive(0, 0, 0, true, false, true);
        } else if (driverController.getRawButton(4)){
            drivetrain.resetNavX();
        } else if (driverController.getRawButton(1)) {
            drivetrain.drive(driverXStick/10, driverYStick/10, driverRotStick/5, false, false, false);
        } else {
            drivetrain.drive(driverXStick, driverYStick, driverRotStick + .01, true, false, false); //the rotation being .01% is so we have a holding position in the rotate position, so the wheels are all good, but there's not enough power to actually drive it. 
        }
    }

<<<<<<< HEAD
    public void startLEDs() {
        m_driverLED = new AddressableLED(1);
        m_driverLEDBuffer = new AddressableLEDBuffer(30); //the int here is how many LEDs there are?
        m_driverLED.setLength(m_driverLEDBuffer.getLength());
        m_driverLED.setData(m_driverLEDBuffer);
        m_driverLED.start();

        //example from WPILIB
        // Create an LED pattern that sets the entire strip to solid red
        //LEDPattern red = LEDPattern.solid(Color.kRed);
        // Apply the LED pattern to the data buffer
        //red.applyTo(m_ledBuffer);
        // Write the data to the LED strip
        //m_led.setData(m_ledBuffer);
    }

    public void runLEDs() {
        LEDPattern red = LEDPattern.solid(Color.kRed);
        LEDPattern green = LEDPattern.solid(Color.kGreen);

        if(driverController.getRawButton(6)) {
            green.applyTo(m_driverLEDBuffer);
        } else {
            red.applyTo(m_driverLEDBuffer);
        }
        m_driverLED.setData(m_driverLEDBuffer);
    }

=======
>>>>>>> 102c2cffaed37d7f0170e7edc9bd55d1a9b7e9f4
    public void letOperatorCookUpdated() {
        //determines/switches mode
        if(operatorController.getRawButton(6)) { //right bumper
            manualOperateElevator = false;
        }
        if(operatorController.getRawButton(5)) { //left bumper
            manualOperateElevator = true;
        }
        //Main logic
        if(manualOperateElevator) {
            m_Elevator.driveMotorNoPID(Math.pow((operatorController.getRawAxis(1)),3), true);
        } else {
            //offset from robot bottom to the bottom of the cradle is 5 inches, will have to factor this in. ... or will we?
            if(operatorController.getRawButton(4)) {
                positionRotations = 34.5 - elevatorOffset; //this worked for L2 before: 41-10.5, goes to 33 inches
                System.out.println("Y");
            } else if (operatorController.getRawButton(2)) {
                System.out.println("B");
                positionRotations = 48.25 - elevatorOffset; //41 -10.5 + 16.5 this is what it was before, i think it worked?
            } else if (operatorController.getRawButton(3)) {
                System.out.println("X");
                positionRotations = 72.5 - elevatorOffset; //should be l4?
            } else if (operatorController.getRawButton(1)) {
                System.out.println("A");
                positionRotations = 0; //no offset, we want to start at the 5.5 inches up
            }
            m_Elevator.driveMotor(positionRotations);

        }
        //axis 2 is left, axis 3 is right

        if(operatorController.getRawAxis(3) > .2) {
            m_Cradle.driveMotorNoPID(Math.pow(operatorController.getRawAxis(3), 3), false);
        } else if (operatorController.getRawAxis(2) > .2) {
            m_Cradle.driveMotorNoPID(Math.pow(operatorController.getRawAxis(2), 3), true);
        } else {
            m_Cradle.driveMotorNoPID(0, false);
        }
    }

    

    public void manualAuto(double driveRots, double turnRots, boolean onBlueAlliance) {
        if(onBlueAlliance) {
        drivetrain.driveManualAuto(driveRots, turnRots);
        } else {
            drivetrain.driveManualAuto(-driveRots, turnRots);
        }
    }

    public void resetDriveEncoder() {
        drivetrain.resetDriveEncoders();
    }

    public void autopath() {
        //3 inch is the width of the entire bumper. 30 inches offset bc our back wheels will start on the line, 27 from center of wheel to other edge of chassis, and 3 inches with bumper
        //88 inches - 30 inches = 58 inches
<<<<<<< HEAD
        manualAuto(53, (.5*Math.PI), true); //never switch to true //ALWAYS SWITCH TO TRUE (check again if you rezero offsets for wheels idk why)
        
=======
        manualAuto(53, (.5*Math.PI), true); //never switch to true //ALWAYS SWITCH TO TRUE I LOST AN AUTO BC OF THIS ITS JOVER YALL
        // if (mTimer.get() > 6) {
        //     m_Elevator.driveMotor(19);
        // } if (mTimer.get() > 9) {
        //     m_Cradle.driveMotorNoPID(1, false);
        // } if (mTimer.get() >10) {
        //     m_Cradle.driveMotorNoPID(0, false);
        // }
>>>>>>> 102c2cffaed37d7f0170e7edc9bd55d1a9b7e9f4
        if(mTimer.get() > 5) {
            m_Elevator.driveMotor(72.5 - elevatorOffset);
        }
        if(mTimer.get()>9) {
            m_Cradle.driveMotorNoPID(.75, false);
        }
        if(mTimer.get()>13) {
            m_Cradle.driveMotorNoPID(0, false);
        }

    }

    public void startTimer() {
        mTimer.start();
    }
    public void resetTimer() {
        mTimer.reset();
    }
    public void stopTimer() {
        mTimer.stop();
    }
}