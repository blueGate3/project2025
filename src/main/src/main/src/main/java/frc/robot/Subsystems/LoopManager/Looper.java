package frc.robot.Subsystems.LoopManager;

import java.util.ArrayList;

public class Looper implements Loop{
    public static Looper instance;
    private final ArrayList<Loop> procedures = new ArrayList<>();
    public static Looper getInstance(){
        if (instance == null){
            instance = new Looper();
        }
        return instance;
    }
    public synchronized void register(Loop procedure){
        procedures.add(procedure);
    }
    public synchronized void run(){
        for (Loop loop: procedures){
            loop.onLoop();
        }
    }
    public synchronized void onStart(){}
    public synchronized void onLoop(){}
    public synchronized void onStop(){}
}