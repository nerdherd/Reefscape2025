package frc.robot.subsystems;

import frc.robot.Constants.BannerSensorConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class BannerSensor implements Reportable {
    private final int blackPort;
    private final int whitePort;
    private final DigitalInput bannerSensorBlack;
    private final DigitalInput bannerSensorWhite;

    private boolean pieceDetected;
    private boolean lastBlackValue;
    private boolean lastWhiteValue;
    private boolean illegalInput = false;

    public BannerSensor() {
        blackPort = BannerSensorConstants.blackPort;
        whitePort = BannerSensorConstants.whitePort;
        bannerSensorBlack = new DigitalInput(blackPort);
        bannerSensorWhite = new DigitalInput(whitePort);

    }

    public boolean pieceDetected() {
        lastBlackValue = bannerSensorBlack.get();
        lastWhiteValue = bannerSensorWhite.get();
        if ((lastBlackValue && lastWhiteValue) || (!lastBlackValue && !lastWhiteValue)) {
            illegalInput = true;
            pieceDetected = false;
        }

        if(!lastBlackValue && lastWhiteValue){
            pieceDetected = true;
        }
        else if(lastBlackValue && !lastWhiteValue){
            pieceDetected = false;
        }
        else{
            DriverStation.reportError("Fault in banner sensor, error code: ", true);
            pieceDetected = false;
        }
        return pieceDetected;
    }

    public boolean pieceDetectedWithoutPolling() {
        return pieceDetected;
    }

    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {}

    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        ShuffleboardTab tab = Shuffleboard.getTab("Indexer");
        tab.addBoolean("Note Detected", this::pieceDetected);
        tab.addBoolean("Banner Sensor Connected", () -> !illegalInput);
        tab.addBoolean("Last Black Value", () -> lastBlackValue);
        tab.addBoolean("Last White Value", () -> lastWhiteValue);
    }
    
}