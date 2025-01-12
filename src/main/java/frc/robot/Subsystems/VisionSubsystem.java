package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    private double xAngleOffset = limelight.getEntry("tx").getDouble(0.0);
    private double yAngleOffset = limelight.getEntry("ty").getDouble(0.0);
    private double area = limelight.getEntry("ta").getDouble(0.0);
    private double latency = limelight.getEntry("tl").getDouble(0.0);
    private double[] botPose = limelight.getEntry("botpose").getDoubleArray(new double[6]); 
    /*
     * botPose (Units meters for distances, Units degrees for rotation):
     * botPose[0] -> X 
     * botPose[1] -> Y 
     * botPose[2] -> Z 
     * botPose[3] -> Roll 
     * botPose[4] -> Pitch 
     * botPose[5] -> Yaw
     */
    public enum ledStatus {
        kBlinking,
        kStatic,
        kDisabled
    }
    public ledStatus ledState = ledStatus.kStatic;
    public void ledStatus(ledStatus ledState){
        this.ledState = ledState;
        limelight.getEntry("ledMode").setNumber(switch(ledState){
            case kBlinking -> 2;
            case kStatic -> 3;
            case kDisabled -> 1;
        });
    }
    public void updateDashboard(){
        SmartDashboard.putNumber("xAngleOffset", xAngleOffset);
        SmartDashboard.putNumber("yAngleOffset", yAngleOffset);
        SmartDashboard.putNumber("Area", area);
        SmartDashboard.putNumber("Latency", latency);
        SmartDashboard.putNumberArray("BotPose", botPose);
        
    }

}
