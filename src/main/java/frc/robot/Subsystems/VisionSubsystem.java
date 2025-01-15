package frc.robot.Subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    private double xAngleOffset = limelight.getEntry("tx").getDouble(0.0);
    private double yAngleOffset = limelight.getEntry("ty").getDouble(0.0);
    private double area = limelight.getEntry("ta").getDouble(0.0);
    private double latency = limelight.getEntry("tl").getDouble(0.0);
    private double[] botPose = limelight.getEntry("botpose").getDoubleArray(new double[6]); 

}
