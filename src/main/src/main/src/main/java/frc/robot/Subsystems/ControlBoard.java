package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Subsystems.LoopManager.*;
public class ControlBoard {
    private final Joystick operatorJoystick = new Joystick(1);
    private final Joystick driveJoystick = new Joystick(0);
    private final Looper driveLooper = Looper.getInstance();
    public ControlBoard(){
        bindButtons();
        driveBinding();
    }
    private void bindButtons(){
        // No buttons to bind currently
    }
    private void driveBinding(){
        Loop runnableDrive = new Loop() {
            @Override
            public void onStart() {
                // TODO Auto-generated method stub
                
            }
            @Override
            public void onStop() {
                // TODO Auto-generated method stub
                
            }
            @Override
            public void onLoop() {
                // Jack add drive commands here, note from brayden 😘
            }
        };
       driveLooper.register(runnableDrive);
    }
}
