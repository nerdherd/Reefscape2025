package frc.robot.subsystems.imu;

import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Constants;
import frc.robot.subsystems.Reportable.LOG_LEVEL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PigeonV2 extends SubsystemBase {
    private Pigeon2 pigeon;
    private double offset, pitchOffset, rollOffset = 0;

    public PigeonV2(int id, String CAN_Name) {
        try {
            this.pigeon = new Pigeon2(id, CAN_Name); // TODO check if canviore names are different
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating Pigeon 2 over CAN: " + ex.getMessage(), true);
        }
        offset = 0;
        pitchOffset = 0;
        rollOffset = 0;
    }

    private void zeroAll() {
        zeroHeading();
        zeroPitch();
        zeroRoll();
    }
    
    private void zeroHeading() {
        pigeon.setYaw(0);
        offset = 0;
    }

    /**
     * Return the internal pigeon object.
     * @return
     */
    private Pigeon2 getPigeon() {
        return this.pigeon;
    }

    private void zeroPitch() {
        this.pitchOffset = -pigeon.getPitch().getValueAsDouble();
    }
    
    private void zeroRoll() {
        this.rollOffset = pigeon.getRoll().getValueAsDouble();
    }

    private void setOffset(double offset) {
        this.offset = offset;
    }

    private void setPitchOffset(double offset) {
        this.pitchOffset = offset;
    }

    private void setRollOffset(double offset) {
        this.rollOffset = offset;
    }

    private void resetHeading(double headingDegrees) {
        zeroHeading();
        offset = headingDegrees;
    }

    private void resetPitch(double pitchDegrees) {
        this.pitchOffset = this.getPitch() - pitchDegrees;
    }

    private void resetRoll(double rollDegrees) {
        this.rollOffset = this.getRoll() - rollDegrees;
    }

    public double getHeading() {
        //return -(pigeon.getAngle() - offset);
        double currentHeading = pigeon.getYaw().getValueAsDouble() - offset; 
        return normalizeToMinus180To180(currentHeading);
    }
    
    private double normalizeToMinus180To180(double angle) {
        angle = (angle % 360 + 360) % 360; // Normalize to [0, 360)
        return (angle > 180) ? angle - 360 : angle; // Convert to (-180, 180)
    }

    private void setHeading(double heading) {
        this.offset += (heading - getHeading());
    }

    private double getYaw() {
        return (pigeon.getYaw().getValueAsDouble() - offset) % 360;
    }

    private double getPitch() {
        return (-pigeon.getPitch().getValueAsDouble() - pitchOffset) % 360;
    }

    private double getRoll() {
        return (pigeon.getRoll().getValueAsDouble() - rollOffset) % 360;
    }

    private double getHeadingOffset() {
        return this.offset;
    }

    private double getRollOffset() {
        return this.rollOffset;
    }

    private double getPitchOffset() {
        return this.pitchOffset;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }
    
    /**
     * For orientations, see page 20 of {@link https://store.ctr-electronics.com/content/user-manual/Pigeon2%20User%27s%20Guide.pdf}
     */
    public Rotation3d getRotation3d() {
        return new Rotation3d(
            Math.toRadians(getRoll()),
            Math.toRadians(getPitch()),
            Math.toRadians(getHeading())
        );
    }
    
    // public void reportToSmartDashboard(LOG_LEVEL level) {
    //     switch (level) {
    //         case OFF:
    //             break;
    //         case ALL:
    //             SmartDashboard.putNumber("Pigeon Firmware Version", pigeon.getVersion().getValue());
    //         case MEDIUM:
    //             SmartDashboard.putNumber("Robot Yaw", this.getYaw());
    //             SmartDashboard.putNumber("Robot Pitch", this.getPitch());
    //             SmartDashboard.putNumber("Robot Roll", this.getRoll());
    //         case MINIMAL:
    //             SmartDashboard.putNumber("Robot Heading", getHeading());
    //     }
    // }
    
    public void initShuffleboard(LOG_LEVEL level) {
        if (level == LOG_LEVEL.OFF)  {
            return;
        }
        ShuffleboardTab tab;
        if (level == LOG_LEVEL.MINIMAL) {
            tab = Shuffleboard.getTab("Main");
        } else {
            tab = Shuffleboard.getTab("Imu");
        }
        switch (level) {
            case OFF:
                break;
            case ALL:
                tab.addNumber("Pigeon Firmware Version", () -> pigeon.getVersion().getValue());
            case MEDIUM:
                tab.addNumber("Robot Yaw", this::getYaw);
                tab.addNumber("Robot Pitch", this::getPitch);
                tab.addNumber("Robot Roll", this::getRoll);
            case MINIMAL:
                tab.addNumber("Robot Heading", this::getHeading);
        }
    }
    
}