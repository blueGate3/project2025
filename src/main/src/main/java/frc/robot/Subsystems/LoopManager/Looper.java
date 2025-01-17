package frc.robot.Subsystems.LoopManager;

public class Looper implements ILooper {
    public static Looper instance;
    private Looper(){}
    public static Looper getInstance(){
        if (instance == null){
            instance = new Looper();
        }
        return instance;
    }
    public void register(Runnable loop){}
    public void start(){}
    public void stop(){}
}
