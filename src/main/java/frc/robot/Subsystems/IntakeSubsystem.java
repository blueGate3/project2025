package frc.robot.Subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax rightUpperMotor = new CANSparkMax(IntakeConstants.kRightUpperMotor, MotorType.kBrushless);
  private final CANSparkMax rightLowerMotor = new CANSparkMax(IntakeConstants.kRightLowerMotor, MotorType.kBrushless);
  private final CANSparkMax leftUpperMotor = new CANSparkMax(IntakeConstants.kLeftUpperMotor, MotorType.kBrushless);
  private final CANSparkMax leftLowerMotor = new CANSparkMax(IntakeConstants.kLeftLowerMotor, MotorType.kBrushless);
  public void setIntakeState(){}
}
