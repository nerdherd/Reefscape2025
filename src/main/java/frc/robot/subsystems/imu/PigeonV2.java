package frc.robot.subsystems.imu;

import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PigeonV2 extends SubsystemBase implements Gyro {
    private Pigeon2 pigeon;
    private double offset, pitchOffset, rollOffset = 0;

    public PigeonV2(int id, String canbusName) {
        this.pigeon = new Pigeon2(id, canbusName);
        offset = 0;
        pitchOffset = 0;
        rollOffset = 0;
    }

    public void zeroAll() {
        zeroHeading();
        zeroPitch();
        zeroRoll();
    }
    
    public void zeroHeading() {
        pigeon.setYaw(0);
        offset = 0;
    }

    /**
     * Return the internal pigeon object.
     * @return
     */
    public Pigeon2 getPigeon() {
        return this.pigeon;
    }

    public void zeroPitch() {
        this.pitchOffset = -pigeon.getPitch().getValueAsDouble();
    }
    
    public void zeroRoll() {
        this.rollOffset = pigeon.getRoll().getValueAsDouble();
    }

    public void setOffset(double offset) {
        this.offset = offset;
    }

    public void setPitchOffset(double offset) {
        this.pitchOffset = offset;
    }

    public void setRollOffset(double offset) {
        this.rollOffset = offset;
    }

    public void resetHeading(double headingDegrees) {
        zeroHeading();
        offset = headingDegrees;
    }

    public void resetPitch(double pitchDegrees) {
        this.pitchOffset = this.getPitch() - pitchDegrees;
    }

    public void resetRoll(double rollDegrees) {
        this.rollOffset = this.getRoll() - rollDegrees;
    }

    public double getHeading() {
        return -(pigeon.getAngle() - offset);
    }

    public void setHeading(double heading) {
        this.offset += (heading - getHeading());
    }

    public double getYaw() {
        double currentYaw = (pigeon.getYaw().getValueAsDouble() - offset) % 360;
        if (currentYaw < 0) {
            return currentYaw + 360;
        } else {
            return currentYaw;
        }
    }

    public double getPitch() {
        return (-pigeon.getPitch().getValueAsDouble() - pitchOffset) % 360;
    }

    public double getRoll() {
        return (pigeon.getRoll().getValueAsDouble() - rollOffset) % 360;
    }

    public double getHeadingOffset() {
        return this.offset;
    }

    public double getRollOffset() {
        return this.rollOffset;
    }

    public double getPitchOffset() {
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

    public double getRate()
    {
        return -1*pigeon.getAngularVelocityZWorld().getValueAsDouble();
    }
    
    public void reportToSmartDashboard(LOG_LEVEL level) {
        switch (level) {
            case OFF:
                break;
            case ALL:
                SmartDashboard.putNumber("Pigeon Firmware Version", pigeon.getVersion().getValue());
            case MEDIUM:
                SmartDashboard.putNumber("Robot Yaw", this.getYaw());
                SmartDashboard.putNumber("Robot Pitch", this.getPitch());
                SmartDashboard.putNumber("Robot Roll", this.getRoll());
            case MINIMAL:
                SmartDashboard.putNumber("Robot Heading", getHeading());
        }
    }
    
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