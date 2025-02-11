// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import 
edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.*;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;

/**
 * NOTE
 * BRAYDEN
 * THIS IS JUST TEMPORARY WHILE I FIX EVERYTHING
 * COZ I WAS HALFWAY THROUGH STUFF LAST TIME I WANTED TO
 * JUST HAVE ANOTHER THING I CAN LOOK AT, SHOULD BE GONE BY NEXT MEETING HOPEFULLY.
 */

/**
 * This is the code to run a single swerve module <br><br>
 * It is called by the Drivetrain subsysem
 */
public class SwerveModule extends SubsystemBase {
        
        private static final double kWheelDiameter = .1016; // 0.1016 M wheel diameter (4"), used to be 4 inches if this breaks it look here
        private static final double kWheelCircumference = Math.PI * kWheelDiameter;
        private static final double rpmToVelocityScaler = (kWheelCircumference / 6.12) / 60; //SDS Mk3 standard gear ratio from motor to wheel, divide by 60 to go from secs to mins
        private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

        private SparkFlex m_driveMotor;
        private SparkFlexConfig m_driveMotorConfig;
        private RelativeEncoder m_driveEncoder;
        private SparkMax m_turningMotor;
        private SparkMaxConfig m_turningMotorConfig;
        public AbsoluteEncoder m_turningEncoder;

        public DutyCycle turnEncoderDutyCycle;
        private int encoderChannelNumber;
        private SparkClosedLoopController m_turnController;

        /**
         * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
         *
         * @param driveMotorChannel CAN ID for the drive motor.
         * @param turningMotorChannel CAN ID for the turning motor.
         * @param driveInverted drive the motor inverted, used for testing
         * @param turnInverted turn motor inverted, used for testing.
         */
        
        public SwerveModule(int driveMotorChannel, int turningMotorChannel, int encoderChannel, double offset, boolean driveInverted, boolean turnInverted) {
            m_driveMotor = new SparkFlex(driveMotorChannel, SparkLowLevel.MotorType.kBrushless);
            m_turningMotor = new SparkMax(turningMotorChannel, SparkLowLevel.MotorType.kBrushless);
            m_driveMotorConfig = new SparkFlexConfig();
            m_turningMotorConfig = new SparkMaxConfig(); 
            int encoderChannelNumber = encoderChannel;
            //TODO update to throughbore encoders

            /** from revlib coding example:
             * The RestoreFactoryDefaults method can be used to reset the configuration parameters
             * in the SPARK MAX to their factory default state. If no argument is passed, these
             * parameters will not persist between power cycles
             */
            //m_motor.restoreFactoryDefaults(); TODO look into setting up configs for this.

            //drive setup, neo 550 with SPARK MAX motor controllers, relative encoders are the ones built into the motor.
            m_driveEncoder = m_driveMotor.getEncoder(); //neo built in encoder
            m_driveMotorConfig.inverted(driveInverted);
            m_driveMotorConfig.closedLoopRampRate(1); //.1 seconds until max speed

            //turning motor setup, using rev throughbore encoders, brushless neo 1.1s, and built in encoders.
            m_turningMotorConfig.inverted(turnInverted);

            // m_turningEncoder = m_turningMotor.getAbsoluteEncoder(); readd after
            turnEncoderDutyCycle = new DutyCycle(new DigitalInput(encoderChannel)); //if we are doing what we had last year

            m_turningMotorConfig.closedLoop
                    .p(.3) //TODO eventually update these from constants 
                    .i(0.0) 
                    .d(0.01)
                    .outputRange((-Math.PI), Math.PI);
                //TODO tune PID, this is what we used last year
            //m_turningMotorConfig.closedLoop
            //        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

            m_driveMotor.configure(m_driveMotorConfig, ResetMode.kResetSafeParameters, null);
            m_turningMotor.configure(m_turningMotorConfig, ResetMode.kResetSafeParameters, null);
            
            m_turnController = m_turningMotor.getClosedLoopController();
            System.out.println("Encoder Channel: " + encoderChannel + ", Encoder Value: "+ (turnEncoderDutyCycle.getOutput()));
            
        }

        /**
         * Sets the desired state for the module.
         * @param desiredState Desired state with speed and angle.
         */
        public void setDesiredState(SwerveModuleState desiredState) {
            //double encoderValue = m_turningEncoder.getPosition();
            double encoderValue = turnEncoderDutyCycle.getOutput();
            // Optimize the reference state to avoid spinning further than 90 degrees
            //SwerveModuleState state = SwerveModuleState.optimize(desiredState, Rotation2d.fromRotations(encoderValue)); //TODO need to get a new way to get state variable
            SwerveModuleState state = new SwerveModuleState(desiredState.speedMetersPerSecond, Rotation2d.fromRotations(desiredState.angle.getRotations()));
            state.optimize(Rotation2d.fromRotations(encoderValue)); 

            m_turnController.setReference(state.angle.getRadians(), ControlType.kPosition);//my code TODO may need to factor in gear ratio

            double drivePower = state.speedMetersPerSecond; 
            m_driveMotor.set(drivePower/10); 
        }

        // public double getTurnEncoder () {
        //     return m_turningEncoder.getPosition() ;
        // }
         
        /**
         * Returns the current state of the module.
         *
         * @return The current state of the module.
         */
        public SwerveModuleState getState() {
            //the getVelocity() function normally returns RPM but is scaled in the SwerveModule constructor to return actual wheel speed
            return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(getTurnEncoderRadians()) );
        }

        public SwerveModuleState getDifferentState() { //TIMES 60 TO CONVERRT FROM MINUTES TO SECONDS
            return new SwerveModuleState((m_driveEncoder.getPosition())*rpmToVelocityScaler*60, new Rotation2d(getTurnEncoderRadians()));
        }

        /**
         * Applies the absolute encoder offset value and converts range
         * from 0-1 to 0-2 pi radians
         * @return Angle of the absolute encoder in radians
         */
        public double getTurnEncoderRadians() {
            //if (turnPWMChannel == 18) { 
                //System.out.println("Encoder Value "+ (turnEncoderDutyCycle.getOutput()));
            //} 
            return turnEncoderDutyCycle.getOutput() * 2 * Math.PI;
        }
}