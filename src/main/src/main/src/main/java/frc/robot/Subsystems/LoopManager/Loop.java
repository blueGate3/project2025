package frc.robot.Subsystems.LoopManager;

public interface Loop {
    public void register(Loop loop);
    public void onStart();
    public void onLoop();
    public void onStop();
}
