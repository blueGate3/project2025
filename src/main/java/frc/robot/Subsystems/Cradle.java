package frc.robot.Subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Cradle {
    private SparkMax m_leftCradleMotor;
    private SparkMaxConfig m_leftCradleMotorConfig;
    private SparkMax m_rightCradleMotor;
    private SparkMaxConfig m_rightCradleMotorConfig;

    public Cradle() {
        m_leftCradleMotor = new SparkMax(18, SparkLowLevel.MotorType.kBrushless);
        m_leftCradleMotorConfig = new SparkMaxConfig();
        m_leftCradleMotorConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(40);
        m_leftCradleMotor.configure(m_leftCradleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        
        m_rightCradleMotor = new SparkMax(15, SparkLowLevel.MotorType.kBrushless); //hear me out: "13 Reasons Why: The Musical"
        m_rightCradleMotorConfig = new SparkMaxConfig();
        m_rightCradleMotorConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(40);
        m_rightCradleMotor.configure(m_rightCradleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    }

    public void driveMotorNoPID(double power, boolean reversed) {
      //power -= .6; //corrects for the fact that triggers start at .5 readout, .6 is just to be safe. 
      power *= .3; //sets max speed at .1
      if(power >= 0) { //only runs if triggers pressed
        if(reversed) {
          m_leftCradleMotor.set(-power);
          m_rightCradleMotor.set(power);
        } else {
          m_leftCradleMotor.set(power);
          m_rightCradleMotor.set(-power);
        }
      }
    }

}
