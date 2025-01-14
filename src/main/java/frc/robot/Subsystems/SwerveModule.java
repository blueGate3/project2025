// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.*;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.*;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;

/**
 * This is the code to run a single swerve module <br><br>
 * It is called by the Drivetrain subsysem
 */
public class SwerveModule extends SubsystemBase {
        
    private static final double kWheelDiameter = .1016; // 0.1016 M wheel diameter (4"), used to be 4 inches if this breaks it look here
        private static final double kWheelCircumference = Math.PI * kWheelDiameter;
        private static final double rpmToVelocityScaler = (kWheelCircumference / 6.12) / 60; //SDS Mk3 standard gear ratio from motor to wheel, divide by 60 to go from secs to mins
        //kWheelCircumference used to be 
        //private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed; // radians per second
        private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

        private final SparkMax m_driveMotor;
        private final SparkMax m_turningMotor;
        private SparkClosedLoopController turnPID;
        private final AbsoluteEncoder m_AbsoluteEncoder;
        private final AbsoluteEncoderConfig m_AbsoluteEncoderConfig;
        private SparkMaxConfig m_driveMotorConfig;
        private SparkMaxConfig m_turningMotorConfig;

        //private final PIDController m_drivePID;
        private final ClosedLoopConfig m_turnPIDConfig;

        private final RelativeEncoder m_driveEncoder;
        //private final DigitalInput m_TurnEncoderInput; used to be two until Brayden suggested combining directly into duty cycle
        private final DutyCycle m_TurnPWMEncoder;

        private double turnEncoderOffset;
        private double encoderBias = 0; //encoder stuff for rotation
        private int turnPWMChannel;

        private final PIDController m_turningPIDController = new PIDController(0.4, 0, 0.01); //HOPEFULLY THIS COMMENT FLAGS ME DOWN BECAUSE THIS IS THE MOST IMPORTANT THING IF YOU READ FROM THE SIDE THIS IS PID also k is generally .45yy
        
        /**
         * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
         *
         * @param driveMotorChannel CAN ID for the drive motor.
         * @param turningMotorChannel CAN ID for the turning motor.
         * @param driveEncoder DIO input for the drive encoder channel A
         * @param turnEncoderPWMChannel DIO input for the drive encoder channel B
         * @param turnOffset offset from 0 to 1 for the home position of the encoder
         */
        
        public SwerveModule(int driveMotorChannel, int turningMotorChannel, int turnEncoderPWMChannel, double turnOffset, boolean driveInverted, boolean turnInverted) {
            
            
            m_AbsoluteEncoderConfig = new AbsoluteEncoderConfig();

            // can spark max motor controller objects
            m_driveMotor = new SparkMax(driveMotorChannel, SparkLowLevel.MotorType.kBrushless);
            m_turningMotor = new SparkMax(turningMotorChannel, SparkLowLevel.MotorType.kBrushless);
            turnPID = m_turningMotor.getClosedLoopController();

            m_driveMotorConfig = new SparkMaxConfig();
            m_turningMotorConfig = new SparkMaxConfig();
            m_turnPIDConfig = new ClosedLoopConfig();

            //spark max built-in encoder
            m_driveEncoder = m_driveMotor.getEncoder();
            // m_driveEncoder.setVelocityConversionFactor(rpmToVelocityScaler);
            //PWM encoder from CTRE mag encoders
            // turnPWMChannel = turnEncoderPWMChannel;
            turnEncoderOffset = turnOffset;
            m_TurnPWMEncoder = new DutyCycle(new DigitalInput(turnPWMChannel));

            m_driveMotorConfig.inverted(driveInverted);
            m_turningMotorConfig.inverted(turnInverted);
            m_driveMotorConfig.closedLoopRampRate(0.1);

            m_turningMotorConfig.encoder.velocityConversionFactor(rpmToVelocityScaler);
            //m_turningMotorConfig.encoder.

            m_AbsoluteEncoder = new AbsoluteEncoder() {
                public double getPosition() {
                    return m_TurnPWMEncoder.getOutput();
                }

                @Override
                public double getVelocity() {
                    // TODO work on this
                    return m_turningMotor.get(); //TODO output value is between 0 and 1, will need to scale
                    //(or at least figure out if we NEED to scale in the first place, may need the 0-1 value for something else later on.)
                }

            };
            m_turnPIDConfig.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
            m_turnPIDConfig.pid(.4, 0, .01); //TODO test to find what values work best
            
            
            // Limit the PID Controller's input range between -pi and pi and set the input
            // to be continuous.
            m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI); //this looks fun try 0 instead of pi may solve doubling issue.
            m_turningPIDController.setTolerance(0.01);
            encoderBias = m_driveEncoder.getPosition();
            //System.out.println("Encoder " + Integer.toString(turnPWMChannel) + " "+ (m_TurnPWMEncoder.getOutput()));
            m_driveMotor.configure(m_driveMotorConfig, null, null);
            m_turningMotor.configure(m_turningMotorConfig, null, null);
            
            //TODO figure out how to configure PID
         }
    

         
        /**
         * Returns the current state of the module.
         *
         * @return The current state of the module.
         */
        public SwerveModuleState getState() {
            //the getVelocity() function normally returns RPM but is scaled in the SwerveModule constructor to return actual wheel speed
            return new SwerveModuleState( m_driveEncoder.getVelocity(), new Rotation2d(getTurnEncoderRadians()) );
        }

        public SwerveModuleState getDifferentState() { //TIMES 60 TO CONVERRT FROM MINUTES TO SECONDS
            return new SwerveModuleState((m_driveEncoder.getPosition()-encoderBias)*rpmToVelocityScaler*60, new Rotation2d(getTurnEncoderRadians()));
        }
        /**
         * Sets the desired state for the module.
         *
         * @param desiredState Desired state with speed and angle.
         */
        public void setDesiredState(SwerveModuleState desiredState) {
            double encoderOffset = turnEncoderOffset - m_TurnPWMEncoder.getOutput();
            // Optimize the reference state to avoid spinning further than 90 degrees
            SwerveModuleState state = SwerveModuleState.optimize(desiredState, Rotation2d.fromRotations(encoderOffset));

             m_turningMotor.set(m_turningPIDController.calculate(Rotation2d.fromRotations(encoderOffset).getRadians(), state.angle.getRadians()));

            double drivePower = state.speedMetersPerSecond ; 
            m_driveMotor.set(drivePower);

        }

        /**
         * Applies the absolute encoder offset value and converts range
         * from 0-1 to 0-2 pi radians
         * @return Angle of the absolute encoder in radians
         */
        public double getTurnEncoderRadians() {
            double appliedOffset = (m_TurnPWMEncoder.getOutput() - turnEncoderOffset) % 1;
            //if (turnPWMChannel == 18) { 
                System.out.println("Encoder " + Integer.toString(turnPWMChannel) + " "+ (m_TurnPWMEncoder));
                
            //} 
            
            //SmartDashboard.putNumber("PWMChannel " + Integer.toString(turnPWMChannel), m_TurnPWMEncoder.getOutput());
            return appliedOffset * 2 * Math.PI;
        }


        /**
         * Calculates the closest angle and direction between two points on a circle.
         * @param currentAngle <ul><li>where you currently are</ul></li>
         * @param desiredAngle <ul><li>where you want to end up</ul></li>
         * @return <ul><li>signed double of the angle (rad) between the two points</ul></li>
         */
        public double closestAngleCalculator(double currentAngle, double desiredAngle) {
            double signedDiff = 0.0;
            double rawDiff = currentAngle > desiredAngle ? currentAngle - desiredAngle : desiredAngle - currentAngle; // find the positive raw distance between the angles
            double modDiff = rawDiff % (2 * Math.PI); // constrain the difference to a full circle

            if (modDiff > Math.PI) { // if the angle is greater than half a rotation, go backwards
                signedDiff = ((2 * Math.PI) - modDiff); //full circle minus the angle
                if (desiredAngle > currentAngle) signedDiff = signedDiff * -1; // get the direction that was lost calculating raw diff
            }

            else {
                signedDiff = modDiff;
                if (currentAngle > desiredAngle) signedDiff = signedDiff * -1;
            }
            
            return signedDiff;
        }
}
