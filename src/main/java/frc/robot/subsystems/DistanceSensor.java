package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class DistanceSensor implements Reportable {
    private final AnalogInput distanceSensor;

    public DistanceSensor(int sensorPort) {
        distanceSensor = new AnalogInput(sensorPort);
    }

    public double getDistanceCM(){
        double rawValue = distanceSensor.getValue();
        //return (Units.secondsToMilliseconds(pulseWidthSec)) /58.0;
        double voltageScaleFactor = RobotController.getVoltage5V();
        double currentDistanceCM = rawValue * voltageScaleFactor* 0.125 *0.2142;
        return currentDistanceCM;
    }


    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {}

    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        ShuffleboardTab tab = Shuffleboard.getTab("Distance Sensor");
        tab.addNumber("Raw Value", distanceSensor::getValue);
        tab.addNumber("Distance Reading in cm", this::getDistanceCM);
    }
    
}
