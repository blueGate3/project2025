package frc.robot.Subsystems;

import frc.robot.Constants.DriveConst;
import frc.robot.Constants.ElevatorConst;

import com.revrobotics.spark.*;
import com.revrobotics.*;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.*;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class NewElevator {
  private SparkFlex m_leftMotor;
  private SparkFlex m_rightMotor;
  private SparkFlexConfig m_leftMotorConfig;
  private SparkFlexConfig m_rightMotorConfig;
  private SparkClosedLoopController m_Controller;
  private RelativeEncoder m_Encoder;

  // private ElevatorSim m_ElevatorSim = new ElevatorSim(
  //   ElevatorConst.kV,
  //   ElevatorConst.kA,
  //     new DCMotor(12,
  //     3.65, 
  //     218, 
  //     3.54, 
  //     702.04, //6704 rpm, max for a vortex
  //     2), 
  //   0, //starts at floor
  //   ElevatorConst.L4Height, //max height
  //   true, 
  //   ElevatorConst.homePosition, //starting position, this is the bottom of the chute for the cradle. 
  //   null);
  private ElevatorFeedforward m_ElevatorFeedforward = new ElevatorFeedforward(
    ElevatorConst.kS, 
    ElevatorConst.kG, 
    ElevatorConst.kV);
  public TrapezoidProfile.State currentState = new TrapezoidProfile.State();
    
  private TrapezoidProfile m_Profile = new TrapezoidProfile(
    new Constraints(ElevatorConst.kMaxVelocity, ElevatorConst.kMaxAcceleration));
  
  public NewElevator() {
    m_leftMotor = new SparkFlex(ElevatorConst.leftID, SparkLowLevel.MotorType.kBrushless);
    m_rightMotor = new SparkFlex(ElevatorConst.rightID, SparkLowLevel.MotorType.kBrushless);
    m_leftMotorConfig = new SparkFlexConfig();
    m_rightMotorConfig = new SparkFlexConfig();

    m_leftMotorConfig
    .idleMode(IdleMode.kBrake)
    .inverted(true)
    .smartCurrentLimit(60);
    m_leftMotorConfig.closedLoop
    .pid(1.2, 0, .4)
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .positionWrappingEnabled(true); //may not need or may be actively bad idk

    m_leftMotorConfig.encoder.positionConversionFactor(ElevatorConst.rotationToMeterScaler); //switches us from motor rotations to meters
    m_leftMotor.getEncoder().setPosition(ElevatorConst.homePosition);

    m_rightMotorConfig.follow(ElevatorConst.leftID);
    m_rightMotor.configure(m_rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_Controller = m_leftMotor.getClosedLoopController();
    m_leftMotor.configure(m_leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


  }

  public void setPosition() {

  }


}