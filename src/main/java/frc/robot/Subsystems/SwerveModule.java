// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.spark.*;
import com.revrobotics.*;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This is the code to run a single swerve module <br><br>
 * It is called by the Drivetrain subsysem
 */
public class SwerveModule extends SubsystemBase {
        
        private static final double kWheelDiameter = .1016; // 0.1016 M wheel diameter (4")
        private static final double kWheelCircumference = Math.PI * kWheelDiameter;
        private static final double turningWheelGearRatio = 150/7; //standard steering gear ratio on MK4i 
        private static final double drivingWheelGearRatio = 6.12; //L3 gear ratio for driving, max velocity of 19.3 ft/sec
        private static final double rpmToVelocityScaler = (kWheelCircumference/(drivingWheelGearRatio*60)); //SDS Mk4I standard gear ratio from motor to wheel, divide by 60 to go from secs to mins
        private static final double rotationsToDistanceScaler = kWheelCircumference / drivingWheelGearRatio; //x is number of rotations. 
        /*
        * - m/s to rpm formula: RPM = ((Velocity in m/s)/(circumference)) *60 (you multiply by 60 to convert revolutions per second to revolutions per minute)
        * - with gear ratio: rpm of output = rpm of motor * (gear ratio/output gear teeth)

        */

        private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared
        public static final double kMaxSpeed = 5.88; // 5.88 meters per second or 19.3 ft/s (max speed of SDS Mk4i with Vortex motor)
        private static final double turnEncoderScalar = 2* Math.PI;
        private SparkFlex m_driveMotor;
        private SparkFlexConfig m_driveMotorConfig;
        private RelativeEncoder m_driveEncoder;
        private SparkMax m_turningMotor;
        private SparkMaxConfig m_turningMotorConfig;
        private AbsoluteEncoder m_turningEncoder;
        public double offset;
        private AbsoluteEncoderConfig m_turnEncoderConfig;
        private EncoderConfig m_driveEncoderConfig;


        private SparkClosedLoopController m_turnController;
        private SparkClosedLoopController m_driveController;

        /**
         * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
         *
         * @param driveMotorChannel CAN ID for the drive motor.
         * @param turningMotorChannel CAN ID for the turning motor.
         * @param encoderChannel DIO channel from the roboRIO for the 
         * @param driveInverted drive the motor inverted, used for testing
         * @param turnInverted turn motor inverted, used for testing.
         */
        public SwerveModule(int driveMotorChannel, int turningMotorChannel, boolean driveInverted, boolean turnInverted) {
            /*
             * CHANGES SINCE END OF MEETING SATURDAY
             * -Switched to running encoders through SparkMax's
             * -Throughbore autmatically zeroed so no need for offset
             * -Restructured code to mimic MAXSwerve template bc it's pretty like me
             * -Corrected getTurnEncoderOutput method to not need offset, return true radians, etc. 
             * SOME HELPFUL LINKS FOR ENCODERS:
             * https://docs.revrobotics.com/brushless/spark-max/encoders/alternate-encoder
             * https://docs.revrobotics.com/brushless/spark-max/encoders/absolute 
             * https://docs.revrobotics.com/rev-crossover-products/sensors/tbe/application-examples 
             * https://docs.revrobotics.com/rev-crossover-products/sensors/tbe/specs 
             */

            //DRIVING
            m_driveMotor = new SparkFlex(driveMotorChannel, SparkLowLevel.MotorType.kBrushless);
            m_driveMotorConfig = new SparkFlexConfig();
            m_driveEncoder = m_driveMotor.getEncoder(); //vortex built in encoder

            m_driveMotorConfig
                .inverted(driveInverted)
                .smartCurrentLimit(40)
                .idleMode(IdleMode.kBrake);
            m_driveMotorConfig.encoder
                .positionConversionFactor(rotationsToDistanceScaler) //what we multiply to convert rotations to distance. returns the meters traveled. 
                .velocityConversionFactor(rpmToVelocityScaler); //divided by 60 to get to meters per second i think
            // m_driveMotorConfig.closedLoop
            //     .pidf(0.3, 0.0, 0.001, (1/565)) //1/565 = what REVLIB reccomended for ff for a vortex specifically. 
            //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            //     .outputRange(-1,1);
            
            m_driveController = m_driveMotor.getClosedLoopController();
            m_driveMotor.configure(m_driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            //TURNING
            m_turningMotor = new SparkMax(turningMotorChannel, SparkLowLevel.MotorType.kBrushless);
            m_turningMotorConfig = new SparkMaxConfig(); 

            m_turningEncoder = m_turningMotor.getAbsoluteEncoder();

            m_turningMotorConfig.absoluteEncoder
                .inverted(true)
                .velocityConversionFactor(turnEncoderScalar/60)
                .positionConversionFactor(turnEncoderScalar);
            m_turningMotorConfig
                .idleMode(IdleMode.kBrake)
                .inverted(turnInverted)
                .smartCurrentLimit(40);
            m_turningMotorConfig.closedLoop
                .pid(1, 0.0, 0.01)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) //when using old method, primaryEncoder was the only thing that worked, absoluteEncoder should work tho.
                .positionWrappingEnabled(true) //this and line below it allow for position wrapping between 0 and 2pi radians 
                .positionWrappingInputRange(0, 2*Math.PI)
                .outputRange(-1, 1);

            m_turnController = m_turningMotor.getClosedLoopController();
            m_turningMotor.configure(m_turningMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            System.out.println("Drive Motor CAN ID: " + driveMotorChannel + ", Initial Encoder Value: "+ (getTurnEncoderOutput(false)));
        }

        /**
         * Sets the desired state for the module.
         * @param desiredState Desired state with speed and angle.
         */
        public void setDesiredState(SwerveModuleState desiredState) {
            SwerveModuleState state = new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
            state.optimize(Rotation2d.fromRadians((getTurnEncoderOutput(false))));// Optimize the reference state to avoid spinning further than 90 degrees
            m_driveMotor.set(state.speedMetersPerSecond/10); //will eventually switch to PID below
            //m_driveController.setReference(((state.speedMetersPerSecond)/kWheelCircumference)*60, ControlType.kVelocity); //desired state gives velocity, to convert: rpm = (Velocity(in m/s) * 60)/pi*diameter(aka wheel circumference)
            m_turnController.setReference(state.angle.getRadians(), ControlType.kPosition);//my code TODO may need to factor in gear ratio. Also, used to be state.angle.getRadians()
            
        }

        /**
         * Returns the current state of the module.
         *
         * @return The current state of the module.
         */
        public SwerveModuleState getState() {
            return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(getTurnEncoderOutput(false)));//the getVelocity() function normally returns RPM but is scaled in the SwerveModule constructor to return m/s
        }

        /**
         * used in odometry to get the distance traveled. 
         * @return SwerveModuleState with distance traveled and Rotation2d of turn angle. 
         */
        public SwerveModuleState getDifferentState() { //TIMES 60 TO CONVERRT FROM MINUTES TO SECONDS
            return new SwerveModuleState((m_driveEncoder.getPosition()), new Rotation2d(getTurnEncoderOutput(false))); //the getPosition() is scaled in the constructor to convert rotations into meters traveled. 
        }

        /**
         * gets the encoder readout of the throughbores, either in absolute position (between 0 and 1) or in radians.
         * @param inRadians if true, converts output to radians, otherwise gives actual duty cycle output.
         * @return the encoder position
         */
        public double getTurnEncoderOutput(boolean inRadians) {
            double encoderValue = m_turningEncoder.getPosition(); //TODO run through gear ratio!?!?! if so, see above formulas for how to. 
            if(inRadians) {
                return Math.toRadians((encoderValue*360)); //multiplies by 360 to get degrees, then converts to radians using Math.toRadians (expects degree parameter)
            } else {
                return encoderValue;
            }
        }

}