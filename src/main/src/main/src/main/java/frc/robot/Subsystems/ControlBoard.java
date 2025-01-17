package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Subsystems.LoopManager.Looper;
public class ControlBoard {
    private final Joystick operatorJoystick = new Joystick(1);
    private final Joystick driveJoystick = new Joystick(0);
    private Looper driveLooper;
    public ControlBoard(){
        bindButtons();
    }
    private void bindButtons(){
        // No buttons to bind currently
    }
    private void driveBinding(){
        driveLooper.register(driveLooper);
        driveLooper = new Looper(){
            @Override
            public synchronized void onLoop() {
                // Jack add your drive command and map it to the joystick : from Brayden
                super.onLoop();
            }
        };
    }
}
