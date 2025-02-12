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
 * This is the code to run a single swerve module <br><br>
 * It is called by the Drivetrain subsysem
 */
public class SwerveModule extends SubsystemBase {
        
        private static final double kWheelDiameter = .1016; // 0.1016 M wheel diameter (4"), used to be 4 inches if this breaks it look here
        private static final double kWheelCircumference = Math.PI * kWheelDiameter;
        private static final double turningWheelGearRatio = 150/7; //standard steering gear ratio on MK4i 
        private static final double drivingWheelGearRatio = 6.12; //L3 gear ratio for driving, max velocity of 19.3 ft/sec
        private static final double rpmToVelocityScaler = (kWheelCircumference / drivingWheelGearRatio) / 60; //SDS Mk4I standard gear ratio from motor to wheel, divide by 60 to go from secs to mins
        private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared
        public static final double kMaxSpeed = 5.88; // 5.88 meters per second or 19.3 ft/s (max speed of SDS Mk4i with Vortex motor)

        private SparkFlex m_driveMotor;
        private SparkFlexConfig m_driveMotorConfig;
        private RelativeEncoder m_driveEncoder;
        private SparkMax m_turningMotor;
        private SparkMaxConfig m_turningMotorConfig;
        public AbsoluteEncoder m_turningEncoder;
        public double offset;

        public DutyCycle turnEncoderDutyCycle;
        private SparkClosedLoopController m_turnController;

        /**
         * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
         *
         * @param driveMotorChannel CAN ID for the drive motor.
         * @param turningMotorChannel CAN ID for the turning motor.
         * @param encoderChannel DIO channel from the roboRIO for the 
         * @param driveInverted drive the motor inverted, used for testing
         * @param turnInverted turn motor inverted, used for testing.
         */
        public SwerveModule(int driveMotorChannel, int turningMotorChannel, int encoderChannel, double offset, boolean driveInverted, boolean turnInverted) {
            m_driveMotor = new SparkFlex(driveMotorChannel, SparkLowLevel.MotorType.kBrushless);
            m_turningMotor = new SparkMax(turningMotorChannel, SparkLowLevel.MotorType.kBrushless);
            m_driveMotorConfig = new SparkFlexConfig();
            m_turningMotorConfig = new SparkMaxConfig(); 

            m_driveMotorConfig.inverted(driveInverted);
            m_turningMotorConfig.inverted(turnInverted);
            m_driveMotorConfig.closedLoopRampRate(1); //seconds until max speed reached

            m_driveEncoder = m_driveMotor.getEncoder(); //neo built in encoder
            turnEncoderDutyCycle = new DutyCycle(new DigitalInput(encoderChannel)); //rev throighbore encoder hooked up to the roboRIO

            m_turningMotorConfig.closedLoop
                    .p(.3) 
                    .i(0.0) 
                    .d(0.01)
                    .outputRange(-1, 1); //TODO look into proper values for this
            //m_turningMotorConfig.closedLoop
            //        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

            m_driveMotor.configure(m_driveMotorConfig, ResetMode.kResetSafeParameters, null);
            m_turningMotor.configure(m_turningMotorConfig, ResetMode.kResetSafeParameters, null);
            m_turnController = m_turningMotor.getClosedLoopController();
            System.out.println("Encoder Channel: " + encoderChannel + ", Initial Encoder Value: "+ (turnEncoderDutyCycle.getOutput()));
        }

        /**
         * Sets the desired state for the module.
         * @param desiredState Desired state with speed and angle.
         */
        public void setDesiredState(SwerveModuleState desiredState) {
            // Optimize the reference state to avoid spinning further than 90 degrees
            //SwerveModuleState state = new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle); //TODO look at this line if it breaks, before it was from rotations.
            //state.optimize(Rotation2d.fromRadians(getTurnEncoderOutput(true))); //TODO look at this line if it breaks, before it was from rotations.
            // double drivePower = state.speedMetersPerSecond; 
            // m_driveMotor.set(drivePower/10); 

            desiredState.optimize(Rotation2d.fromRadians(getTurnEncoderOutput(true)));
            m_driveMotor.set(desiredState.speedMetersPerSecond/kMaxSpeed); //divided by max speed so the output cannot be greater than 1. 
            m_turnController.setReference(desiredState.angle.getRotations(), ControlType.kPosition);//my code TODO may need to factor in gear ratio, also used to be state.angle.getRadians()
        }

        /**
         * Returns the current state of the module.
         *
         * @return The current state of the module.
         */
        public SwerveModuleState getState() {
            return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(getTurnEncoderOutput(true)) );//the getVelocity() function normally returns RPM but is scaled in the SwerveModule constructor to return actual wheel speed
        }

        public SwerveModuleState getDifferentState() { //TIMES 60 TO CONVERRT FROM MINUTES TO SECONDS
            return new SwerveModuleState((m_driveEncoder.getPosition())*rpmToVelocityScaler*60, new Rotation2d(getTurnEncoderOutput(true)));
        }

        /**
         * gets the encoder readout of the throughbores, either in absolute position (between 0 and 1) or in radians.
         * @param inRadians if true, converts output to radians, otherwise gives actual duty cycle output.
         * @return the encoder position
         */
        public double getTurnEncoderOutput(boolean inRadians) {
            double encoderValue = turnEncoderDutyCycle.getOutput() - offset;
            if (inRadians) {
                encoderValue = encoderValue * 2 * Math.PI;
            }
            return encoderValue;
        }
}