package frc.robot.Subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

public class Elevator {
    private SparkFlex m_elevatorMotor;
    private SparkFlexConfig m_elevatorMotorConfig;
    private SparkClosedLoopController m_elevatorController;
    private AbsoluteEncoder m_elevatorEncoder;
    private final double elevatorEncoderScalar = (Math.PI * 1.94); //

    public Elevator() {
        m_elevatorMotor = new SparkFlex(9, SparkLowLevel.MotorType.kBrushless);
        m_elevatorMotorConfig = new SparkFlexConfig();
        m_elevatorEncoder = m_elevatorMotor.getAbsoluteEncoder();
    
        m_elevatorMotorConfig.encoder
        .positionConversionFactor(1); //note Prngles Hot Ones collab is very good apparently
        m_elevatorMotorConfig
        .idleMode(IdleMode.kBrake)
        .inverted(true)
        .smartCurrentLimit(40);
        m_elevatorMotor.getEncoder().setPosition(0);

        m_elevatorMotorConfig.closedLoop
        .pid(1.2, 0, .4)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder) //when using old method, primaryEncoder was the only thing that worked, absoluteEncoder should work tho.
        .positionWrappingEnabled(true) //this and line below it allow for position wrapping between 0 and 2pi radians 
        //.positionWrappingInputRange(0, 2*Math.PI)
        .outputRange(-.4, .4);
    
        m_elevatorController = m_elevatorMotor.getClosedLoopController();
        m_elevatorMotor.configure(m_elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    }

    

  /**
   * 
   * @param positionRotations
   */
  public void driveMotor(double heightInches) {
    /*
     * 32 inches = L1 and L2
     * 48 inches = L3
     * 72 inches = L4
     * Diameter of spinny thing = 1.94 inches, so circumference = 2*PI*1.94 per rotation. Gear ratio is 10, so scalar = 6.28*1.94 / 10 
     * currently offset is 8.5
     */
    heightInches *= Math.PI * 1; //times one bc that's the diameter of the spool, just wanted to have it in there to show I did it..
    m_elevatorController.setReference(heightInches, ControlType.kPosition);
  }

  public void driveMotorNoPID(double power, boolean reversed) {
    if(reversed) {
      m_elevatorMotor.set(-power);
    } else {
      m_elevatorMotor.set(power);
    }
  }

  public void getElevatorRotations() {
    System.out.println("Elevator Height:" + (m_elevatorMotor.getEncoder().getPosition()*(Math.PI)/10));
  }
}
