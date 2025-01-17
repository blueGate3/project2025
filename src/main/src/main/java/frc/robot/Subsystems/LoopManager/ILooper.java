package frc.robot.Subsystems.LoopManager;

public interface ILooper {
    public void register(Runnable loop);
    public void start();
    public void stop();
}
