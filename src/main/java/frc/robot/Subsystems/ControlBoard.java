package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
                SmartDashboard.putString("DriveLoop", "Loop for drive commands running.");
            }
            @Override
            public void ILoop() {
                // Jack add drive commands here, note from brayden :kissface:
            }
        };
        driveLooper.register(runnableDrive);
    }
}
