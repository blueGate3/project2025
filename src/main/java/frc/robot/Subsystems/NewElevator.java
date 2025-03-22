package frc.robot.Subsystems;
import com.revrobotics.spark.*;
import com.revrobotics.*;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.*;

public class NewElevator {
    private SparkFlex m_elevatorMotor;
    private SparkFlexConfig m_elevatorMotorConfig;
    private SparkClosedLoopController m_elevatorController;

    public NewElevator() {
        m_elevatorMotor = new SparkFlex(9, SparkLowLevel.MotorType.kBrushless);
        m_elevatorMotorConfig = new SparkFlexConfig();


        m_elevatorMotorConfig
        .idleMode(IdleMode.kBrake)
        .inverted(true)
        .smartCurrentLimit(80); //can't run for too long
        m_elevatorMotor.getEncoder().setPosition(0);

        m_elevatorMotorConfig.closedLoop
        .pid(1.2, 0, .4)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .outputRange(1, 1);
    
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
     * Diameter of spinny thing = 1 inch
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
    System.out.println("ElevatorHeight" + (m_elevatorMotor.getEncoder().getPosition()*Math.PI));
  }
}
