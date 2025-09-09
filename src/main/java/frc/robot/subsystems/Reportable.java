package frc.robot.subsystems;

public interface Reportable {
    public enum LOG_LEVEL {
        ALL,
        MEDIUM,
        MINIMAL,
        OFF
    }

    public void initShuffleboard(LOG_LEVEL priority);
}