package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Rangefinder {
    private AnalogInput m_sensorInput;
    private DigitalOutput m_sensorOutput;
    private double voltScaleFactor = 0;
    private double rangeCM = 0;
    /**
     * Creates a new Ultrasonic Range Finder
     * @param dio The channel on the Analog In part of the RIO, this is where 
     * @param analog The 
     */
    public Rangefinder(int dio, int analog) {
        m_sensorInput = new AnalogInput(analog);
        m_sensorOutput = new DigitalOutput(dio);
    }

    public void turnOn() {
        m_sensorOutput.set(true);
    }

    /**
     * Gets the range of the sensor. Make sure only the sensor you want is on, and the other two are set to off. 
     * @return range in inches.
     */
    public double getRange() {
        voltScaleFactor = 5/RobotController.getBatteryVoltage();
        rangeCM = m_sensorInput.getValue()*voltScaleFactor*.125;
        return rangeCM; //could condense but nah may need it later.
    }

    public void turnOff() {
        m_sensorOutput.set(false);
    }
}
