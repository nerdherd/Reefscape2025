package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;

public class MotorConfigs {
    private final TalonFXConfiguration configs = new TalonFXConfiguration();
    public void apply(TalonFX motor) {
        StatusCode response = motor.getConfigurator().apply(configs);
        if (!response.isOK()){
            DriverStation.reportError("Could not apply motor configs, error code:" + response.toString(), true);
        }
    }

    public MotorConfigs(double kP, double kI, double kD, InvertedValue inversion, double totalRatio) {
        configs.Slot0.kP = kP;
        configs.Slot0.kI = kI;
        configs.Slot0.kD = kD;
        configs.Slot0.kG = 0.0;
        configs.Slot0.kV = 0.0;
        configs.Slot0.kA = 0.0;
        configs.Slot0.kS = 0.0;

        // TODO FIGURE OUT SUPPLY CURRENTS
        configs.CurrentLimits.SupplyCurrentLimit = 70; //40?
        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        configs.CurrentLimits.SupplyCurrentLowerLimit = 40; //45?
        configs.CurrentLimits.SupplyCurrentLowerTime = 1.0; //0.1?
        configs.Feedback.SensorToMechanismRatio = totalRatio;
        configs.MotorOutput.Inverted = inversion;
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // TODO decide on default value
    }

    public MotorConfigs(double kP, double kI, double kD, InvertedValue inversion, double totalRatio, double cruiseVelocity, double acceleration, double jerk) {
        this(kP, kI, kD, inversion, totalRatio);
        withMotionMagic(cruiseVelocity, acceleration, jerk);
    }

    // any additional functions can be added, this is only the bare bones
    // ---------------------- probably will use --------------------------------------
    public MotorConfigs withNeutralMode(NeutralModeValue val) { configs.MotorOutput.NeutralMode = val; return this; }
    public MotorConfigs withkG(double kG) { configs.Slot0.kG = kG; return this;}
    public MotorConfigs withMotionMagic(double cruiseVelocity, double acceleration, double jerk) { 
        configs.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity; 
        configs.MotionMagic.MotionMagicAcceleration = acceleration; 
        configs.MotionMagic.MotionMagicJerk = jerk; 
        return this; 
    }

    // ---------------------- probably won't -----------------------------------------
    public MotorConfigs withkVAS(double kV, double kA, double kS) { configs.Slot0.kV = kV; configs.Slot0.kA = kA; configs.Slot0.kS = kS; return this;}
    public MotorConfigs withFeedback(FeedbackSensorSourceValue v) { configs.Feedback.FeedbackSensorSource = v; return this; }
}
