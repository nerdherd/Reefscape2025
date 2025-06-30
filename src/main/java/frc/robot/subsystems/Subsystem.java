package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Subsystem extends SubsystemBase {
    public enum LOG_LEVEL {
        ALL,
        MEDIUM,
        MINIMAL,
        OFF
    }

    // private final ShuffleboardTab tab = Shuffleboard.getTab(getClass().getName().lastIndexOf('.'));

    public final void initShuffleboard(LOG_LEVEL logLevel) {
        String s = getClass().getName();
        ShuffleboardTab tab = Shuffleboard.getTab(s.substring(s.lastIndexOf('.') + 1).trim());
        switch (logLevel) {
            case ALL:
                initShuffleboardALL(tab);
            case MEDIUM:
                initShuffleboardMEDIUM(tab);
            case MINIMAL:
                initShuffleboardMINIMAL(tab);
            default:
                break;
        }
    }

    abstract public void initShuffleboardALL(ShuffleboardTab tab);
    abstract public void initShuffleboardMEDIUM(ShuffleboardTab tab);
    abstract public void initShuffleboardMINIMAL(ShuffleboardTab tab);
}
