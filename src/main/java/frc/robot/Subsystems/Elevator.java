package frc.robot.Subsystems;

import frc.robot.Robot;
import frc.robot.Constants.DriveConst;
import frc.robot.Constants.ElevatorConst;

import com.revrobotics.spark.*;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.*;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.*;
import edu.wpi.first.util.datalog.*;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.sysid.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;



public class Elevator extends SubsystemBase{
  private SparkFlex m_leftMotor;
  private SparkFlex m_rightMotor;
  private SparkFlexConfig m_leftMotorConfig;
  private SparkFlexConfig m_rightMotorConfig;
  private SparkClosedLoopController m_Controller;
  private double feedForwardValue;
  
  public TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  private TrapezoidProfile m_Profile = new TrapezoidProfile(
    new Constraints(ElevatorConst.kMaxVelocity, ElevatorConst.kMaxAcceleration));

  private ElevatorFeedforward m_ElevatorFeedforward = new ElevatorFeedforward(
    ElevatorConst.kS, 
    ElevatorConst.kG, 
    ElevatorConst.kV); //according to wpilib, kA can be omitted.

  private final MutVoltage m_appliedVoltage = Volts.mutable(0);// Mutable holders for unit-safe linear velocity, acceleration and distance values, persisted to avoid reallocation. from a wpilib example for sysID
  private final MutDistance m_distance = Meters.mutable(0);
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);
  SysIdRoutine routine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(voltage -> {
      m_leftMotor.setVoltage(voltage);
      m_rightMotor.setVoltage(voltage);

    }, log -> {
      // Record a frame for the left motor.
      log.motor("Left Elevator Motor: ")
          .voltage(
              m_appliedVoltage.mut_replace(
                  m_leftMotor.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts)) //may be just getOutput()
          .linearPosition(m_distance.mut_replace(m_leftMotor.getEncoder().getPosition(), Meters))
          .linearVelocity(
              m_velocity.mut_replace(m_leftMotor.getEncoder().getVelocity(), MetersPerSecond));
      // Record a frame for the right motor.
      log.motor("Right Elevator Motor: ")
          .voltage(
              m_appliedVoltage.mut_replace(
                  m_rightMotor.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
          .linearPosition(m_distance.mut_replace(m_rightMotor.getEncoder().getPosition(), Meters))
          .linearVelocity(
              m_velocity.mut_replace(m_rightMotor.getEncoder().getVelocity(), MetersPerSecond));

    }, this)
  );
  



  public Elevator() {
    m_leftMotor = new SparkFlex(ElevatorConst.leftID, SparkLowLevel.MotorType.kBrushless);
    m_rightMotor = new SparkFlex(ElevatorConst.rightID, SparkLowLevel.MotorType.kBrushless);
    m_leftMotorConfig = new SparkFlexConfig();
    m_rightMotorConfig = new SparkFlexConfig();

    m_leftMotorConfig
    .idleMode(IdleMode.kBrake)
    .inverted(true)
    .smartCurrentLimit(ElevatorConst.maxCurrent);

    // m_leftMotorConfig.closedLoop
    //   .maxMotion
    //   .maxVelocity(ElevatorConst.kMaxVelocity)
    //   .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
    //   .maxAcceleration(ElevatorConst.kMaxAcceleration);

    m_leftMotorConfig.closedLoop
      .pid(ElevatorConst.kP, 
      ElevatorConst.kI, 
      ElevatorConst.kD)
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    m_leftMotorConfig.encoder
      .velocityConversionFactor(ElevatorConst.rotationToMeterScaler*60) //times 60 for minutes to seconds, hopefully
      .positionConversionFactor(ElevatorConst.rotationToMeterScaler); //switches us from motor rotations to meters

    m_leftMotor.getEncoder().setPosition(ElevatorConst.homePosition);

    m_Controller = m_leftMotor.getClosedLoopController();
    m_leftMotor.configure(m_leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_rightMotorConfig.follow(ElevatorConst.leftID);
    m_rightMotor.configure(m_rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


  }

  public void setPosition(TrapezoidProfile.State m_goalState) { //double check this is good; https://docs.revrobotics.com/revlib/spark/closed-loop/getting-started-with-pid-tuning#arbitrary-feed-forward
    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints. - wpilib
    m_setpoint = m_Profile.calculate(feedForwardValue, m_setpoint, m_goalState);
    feedForwardValue = m_ElevatorFeedforward.calculate(m_setpoint.velocity); 
    
    m_Controller.setReference(
      m_setpoint.position,
      ControlType.kPosition,
      ClosedLoopSlot.kSlot0,
      feedForwardValue
    );
  }

  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  public void driveMotorNoPID(double power, boolean reversed) {
    if(reversed) {
      m_leftMotor.set(-power);
      m_rightMotor.set(-power);
    } else {
      m_leftMotor.set(power);
      m_rightMotor.set(power);
    }
  }

}