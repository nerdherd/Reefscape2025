package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class DistanceSensor implements Reportable {
    private final Counter distanceSensor;
    public double SensorReading;
    private final String name;

    public DistanceSensor(String name, int sensorPort) {
        this.name = name;

        distanceSensor = new Counter(sensorPort);
    }

    public double getDistanceCM(){
        double pulseWidthSec = distanceSensor.getPeriod();
        return (pulseWidthSec * 1000000) /58.0;
    }


    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {}

    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        ShuffleboardTab tab = Shuffleboard.getTab(name);
        tab.addNumber("Distance Reading in cm", ()-> getDistanceCM());

    }
    
}
