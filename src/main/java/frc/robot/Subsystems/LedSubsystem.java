package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;
import edu.wpi.first.wpilibj2.command.Command;

public class LedSubsystem extends Command {
    private final PWM pwmChannel = new  PWM(0);
    public LedSubsystem(){
        pwmChannel.setPeriodMultiplier(PeriodMultiplier.k1X);
        pwmChannel.setBoundsMicroseconds(2000, 1600, 1500, 1400, 1000); 
    }
    public enum LedState{
        kPARTY_PALETTE,
        kBREATH_RED,
        kGREEN
    }
    private LedState ledState = LedState.kBREATH_RED;
    public void setLedState(LedState state){ 
        pwmChannel.setSpeed(switch(ledState){
            case kPARTY_PALETTE -> -0.97;
            case kBREATH_RED -> -0.17;
            case kGREEN -> 0.77;
        });
    }
}
