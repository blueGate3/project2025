// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.DriveContainer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.revrobotics.spark.*;
import com.revrobotics.*;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
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
public class UpdatedSwerveModule extends SubsystemBase {
        
        private static final double kWheelDiameter = .1016; // 0.1016 M wheel diameter (4"), used to be 4 inches if this breaks it look here
        private static final double kWheelCircumference = Math.PI * kWheelDiameter;
        private static final double rpmToVelocityScaler = (kWheelCircumference / 6.12) / 60; //SDS Mk3 standard gear ratio from motor to wheel, divide by 60 to go from secs to mins
        private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

        private final SparkMax m_driveMotor;
        private SparkMaxConfig m_driveMotorConfig;
        private final RelativeEncoder m_driveEncoder;

        private final SparkFlex m_turningMotor;
        private SparkFlexConfig m_turningMotorConfig;
        private AbsoluteEncoder m_turningEncoder;
        
        private SparkClosedLoopController m_turnController;

        private double turnEncoderOffset;
        private double encoderBias = 0; //encoder stuff for rotation

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
            //drive setup, neo 550 with SPARK MAX motor controllers, relative encoders are the ones built into the motor.
            m_driveMotor = new SparkMax(driveMotorChannel, SparkLowLevel.MotorType.kBrushless);
            m_driveMotorConfig = new SparkMaxConfig();
            m_driveEncoder = m_driveMotor.getEncoder(); //spark max built-in encoder
            m_driveMotorConfig.inverted(driveInverted);
            m_driveMotorConfig.closedLoopRampRate(0.1); //.1 seconds until max speed

            //turning motor setup, using cancoders, spark flexes and neo vortexes.
            m_turningMotor = new SparkFlex(turningMotorChannel, SparkLowLevel.MotorType.kBrushless);
            m_turningMotorConfig = new SparkFlexConfig();
            m_turningMotorConfig.inverted(turnInverted);
            m_turningEncoder = new AbsoluteEncoder() {
                public double getPosition() {
                    m_turningMotor.getAbsoluteEncoder().getPosition();
                }
                public double getVelocity() {
                    m_turningMotor.getAbsoluteEncoder().getVelocity(); //currently in rpm
                }
            };

            m_turnController = new SparkClosedLoopController();
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
            double appliedOffset = (m_TurnPWMEncoder.getOutput() - turnEncoderOffset) % 1; //TODO why mod 1?!?!
            //if (turnPWMChannel == 18) { 
                //System.out.println("Encoder " + Integer.toString(turnPWMChannel) + " "+ (m_TurnPWMEncoder));
                
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