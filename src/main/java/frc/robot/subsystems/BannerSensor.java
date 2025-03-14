package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class BannerSensor implements Reportable {
    private final DigitalInput bannerSensorBlack;
    private final DigitalInput bannerSensorWhite;

    private final String name;

    private boolean detected;
    private boolean lastBlackValue;
    private boolean lastWhiteValue;
    private boolean illegalInput = false;

    public BannerSensor(String name, int blackPort, int whitePort) {
        this.name = name;
        bannerSensorBlack = new DigitalInput(blackPort);
        bannerSensorWhite = new DigitalInput(whitePort);
    }

    public boolean sensorDetected() {
        lastBlackValue = bannerSensorBlack.get();
        lastWhiteValue = bannerSensorWhite.get();
        if ((lastBlackValue && lastWhiteValue) || (!lastBlackValue && !lastWhiteValue)) {
            illegalInput = true;
            detected = false;
        }

        if(!lastBlackValue && lastWhiteValue){
            detected = true;
        }
        else if(lastBlackValue && !lastWhiteValue){
            detected = false;
        }
        else{
            DriverStation.reportError("Fault in banner sensor, error code: ", true);
            detected = false;
        }
        return detected;
    }

    public boolean pieceDetectedWithoutPolling() {
        return detected;
    }

    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {}

    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        ShuffleboardTab tab = Shuffleboard.getTab(name);
        tab.addBoolean("Detected", this::sensorDetected);
        tab.addBoolean("Banner Sensor Connected", () -> !illegalInput);
        tab.addBoolean("Last Black Value", () -> lastBlackValue);
        tab.addBoolean("Last White Value", () -> lastWhiteValue);
    }
    
}