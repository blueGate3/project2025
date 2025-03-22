// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.revrobotics.spark.*;
import com.revrobotics.*;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConst;

/**
 * This is the code to run a single swerve module <br><br>
 * It is called by the Drivetrain subsysem
 * This is such shit code im sorry
 */
public class SwerveModule extends SubsystemBase {
        private SparkFlex m_driveMotor;
        private SparkFlexConfig m_driveMotorConfig;
        private RelativeEncoder m_driveEncoder;
        private SparkMax m_turningMotor;
        private SparkMaxConfig m_turningMotorConfig;
        private AbsoluteEncoder m_turningEncoder;
        public double offset;

        private SparkClosedLoopController m_turnController;
        private SparkClosedLoopController m_driveController;

        /**
         * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
         *
         * @param driveMotorChannel CAN ID for the drive motor.
         * @param turningMotorChannel CAN ID for the turning motor.
         */
        public SwerveModule(int driveMotorChannel, int turningMotorChannel) {
            m_driveMotor = new SparkFlex(driveMotorChannel, SparkLowLevel.MotorType.kBrushless);
            m_driveMotorConfig = new SparkFlexConfig();
            m_driveEncoder = m_driveMotor.getEncoder(); //vortex built in encoder
            m_driveEncoder.setPosition(0); 

            m_driveMotorConfig
                .smartCurrentLimit(55)
                .openLoopRampRate(.25)
                .idleMode(IdleMode.kBrake);

            // m_driveMotorConfig.closedLoop
            //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            //     .outputRange(-.2, .2) //sets max speed to 1/10 of full power
            //     .pid(.3, 0, 0.4);
                
            m_driveController = m_driveMotor.getClosedLoopController();
            m_driveMotorConfig.closedLoop
            .pidf(0.3, 0.0, 0.4, (1/565)) //1/565 = what REVLIB reccomended for ff for a vortex specifically. 
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-1,1);
            
            m_driveMotor.configure(m_driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


            m_turningMotor = new SparkMax(turningMotorChannel, SparkLowLevel.MotorType.kBrushless);
            m_turningMotorConfig = new SparkMaxConfig(); 

            m_turningEncoder = m_turningMotor.getAbsoluteEncoder();

            m_turningMotorConfig.absoluteEncoder
                .inverted(true)
                .velocityConversionFactor(DriveConst.turnEncoderScalar/60)
                .positionConversionFactor(DriveConst.turnEncoderScalar);
            m_turningMotorConfig
                .idleMode(IdleMode.kBrake)
                //.openLoopRampRate(1) //why in the hell was this here?
                .smartCurrentLimit(40);
            m_turningMotorConfig.closedLoop
                .pid(.7, 0.0, 0.05)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) //when using old method, primaryEncoder was the only thing that worked, absoluteEncoder should work tho.
                .positionWrappingEnabled(true) //this and line below it allow for position wrapping between 0 and 2pi radians 
                .positionWrappingInputRange(0, 2*Math.PI)
                .outputRange(-1, 1);

            m_turnController = m_turningMotor.getClosedLoopController();
            m_turningMotor.configure(m_turningMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            System.out.println("Drive Motor CAN ID: " + driveMotorChannel + ", Initial Encoder Value: "+ (getTurnEncoderOutput(true)));
        }

        /**
         * Sets the desired state for the module.
         * @param desiredState Desired state with speed and angle.
         */
        public void setDesiredState(SwerveModuleState desiredState) {
            desiredState.optimize(Rotation2d.fromRadians((getTurnEncoderOutput(false))));// Optimize the reference state to avoid spinning further than 90 degrees
           
            double drivePower = (desiredState.speedMetersPerSecond) * desiredState.angle.minus(new Rotation2d(getTurnEncoderOutput(false))).getCos(); //multiplies drive power by how close we are to our desired angle so we dont tear up the tires.
            //look at cosine compensation with wpilib

            m_driveMotor.set(drivePower); //will eventually switch to PID below
            //m_driveController.setReference(state.speedMetersPerSecond, ControlType.kVelocity); //desired state gives velocity, to convert: rpm = (Velocity(in m/s) * 60)/pi*diameter(aka wheel circumference)
            m_turnController.setReference(desiredState.angle.getRadians(), ControlType.kPosition);//my code TODO may need to factor in gear ratio
        }

        public void driveAutoOnRots(double driverRots, double turnRots) {
            driverRots = (driverRots * ((4*Math.PI) / DriveConst.drivingWheelGearRatio))/5;
            m_driveController.setReference((driverRots), ControlType.kPosition); //2 inch radius wheels, now we have inches conversion
            m_turnController.setReference(turnRots, ControlType.kPosition);
        }

        public void resetDriveEncoder() {
            m_driveMotor.getEncoder().setPosition(0);
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
         * @param inDegrees if true, converts output to degrees, otherwise gives radians
         * @return the encoder position
         */
        public double getTurnEncoderOutput(boolean inDegrees) {
            double encoderValue = m_turningEncoder.getPosition(); //TODO run through gear ratio!?!?! if so, see above formulas for how to. 
            if(inDegrees) {
                return Math.toRadians((encoderValue*360)); //multiplies by 360 to get degrees, then converts to radians using Math.toRadians (expects degree parameter)
            } else {
                return encoderValue;
            }
        }

}