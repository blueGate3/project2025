package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Joystick;

public class ControlBoard {
    private final Joystick operatorJoystick = new Joystick(1);
    private final Joystick driveJoystick = new Joystick(0);
    public ControlBoard(){
        bindButtons();
    }
    private void bindButtons(){
        // No buttons to bind currently
    }
}
