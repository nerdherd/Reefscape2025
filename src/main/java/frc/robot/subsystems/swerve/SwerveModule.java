package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Reportable;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANcoder;

/**
 * Swerve module that uses CANCoder for the absolute position
 */
public class SwerveModule implements Reportable {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANcoder canCoder;
    private final TalonFXConfigurator driveConfigurator;
    private final TalonFXConfigurator turnConfigurator;

    private final DutyCycleOut driveRequest; 
    private final DutyCycleOut turnRequest;
    private final VelocityVoltage driveVelocityRequest;
    private final NeutralOut brakeRequest;

    private final Slot0Configs drivePIDConfigs;

    private final int driveMotorID;
    private final int turnMotorID;
    private final int CANCoderID;

    private final PIDController turningController;
    private final boolean invertTurningEncoder;

    private double currentPercent = 0;
    private double currentTurnPercent = 0;
    private double currentAngle = 0;
    private double desiredAngle = 0;
    private double desiredVelocity = 0;
    private boolean velocityControl = true;

    private SwerveModuleState desiredState = null;
    private SwerveModulePosition currPosition = new SwerveModulePosition();
    private SwerveModuleState currState = new SwerveModuleState();

    /**
     * Construct a new CANCoder Swerve Module.
     * 
     * @param driveMotorId
     * @param turningMotorId
     * @param invertDriveMotor
     * @param invertTurningMotor
     * @param CANCoderId
     * @param CANCoderOffsetDegrees
     * @param CANCoderReversed
     */
    public SwerveModule(int driveMotorId, int turningMotorId, boolean invertDriveMotor, boolean invertTurningMotor, 
        int CANCoderId, boolean CANCoderReversed) {
        this.canCoder = new CANcoder(CANCoderId, ModuleConstants.kCANivoreName);
        this.driveMotor = new TalonFX(driveMotorId, ModuleConstants.kCANivoreName);
        this.turnMotor = new TalonFX(turningMotorId, ModuleConstants.kCANivoreName);
        
        this.driveConfigurator = driveMotor.getConfigurator();
        this.turnConfigurator = turnMotor.getConfigurator();
        
        this.driveRequest = new DutyCycleOut(0);
        this.turnRequest = new DutyCycleOut(0);
        this.driveRequest.EnableFOC = true;
        this.turnRequest.EnableFOC = true;
        
        this.driveVelocityRequest = new VelocityVoltage(0);
        this.driveVelocityRequest.Slot = 0;

        this.drivePIDConfigs = new Slot0Configs();
        this.driveConfigurator.refresh(drivePIDConfigs);

        this.brakeRequest = new NeutralOut();

        this.driveMotorID = driveMotorId;
        this.turnMotorID = turningMotorId;
        this.CANCoderID = CANCoderId;

        this.turningController = new PIDController(
            ModuleConstants.kPTurning,
            ModuleConstants.kITurning,
            ModuleConstants.kDTurning);
        turningController.enableContinuousInput(-Math.PI, Math.PI); // Originally was -pi to pi
        turningController.setTolerance(.005);

        this.driveMotor.setInverted(invertDriveMotor);
        this.turnMotor.setInverted(invertTurningMotor);
        this.invertTurningEncoder = CANCoderReversed;
        
        this.desiredState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

        TalonFXConfiguration driveMotorConfigs = new TalonFXConfiguration();
        driveConfigurator.refresh(driveMotorConfigs);
        driveMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveMotorConfigs.Voltage.PeakForwardVoltage = 11.5;
        driveMotorConfigs.Voltage.PeakReverseVoltage = -11.5;
        driveMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        driveMotorConfigs.MotorOutput.DutyCycleNeutralDeadband = ModuleConstants.kDriveMotorDeadband;
        driveMotorConfigs.CurrentLimits.SupplyCurrentLimit = 10;
        driveMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;
        // driveMotorConfigs.CurrentLimits.SupplyCurrentThreshold = 30;
        // driveMotorConfigs.CurrentLimits.SupplyTimeThreshold = 0.25;
        driveMotorConfigs.CurrentLimits.StatorCurrentLimit = 40;
        driveMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        driveMotorConfigs.Audio.AllowMusicDurDisable = true;
        driveConfigurator.apply(driveMotorConfigs);

        TalonFXConfiguration turnMotorConfigs = new TalonFXConfiguration();
        turnConfigurator.refresh(turnMotorConfigs);
        turnMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        turnMotorConfigs.Feedback.FeedbackRemoteSensorID = CANCoderId;
        turnMotorConfigs.Feedback.RotorToSensorRatio = 150.0/7.0;
        turnMotorConfigs.Feedback.SensorToMechanismRatio = 1;
        turnMotorConfigs.ClosedLoopGeneral.ContinuousWrap = true;
        turnMotorConfigs.Voltage.PeakForwardVoltage = 11.5;
        turnMotorConfigs.Voltage.PeakReverseVoltage = -11.5;
        turnMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        turnMotorConfigs.MotorOutput.DutyCycleNeutralDeadband = ModuleConstants.kDriveMotorDeadband;
        turnMotorConfigs.CurrentLimits.SupplyCurrentLimit = 25;
        turnMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        // turnMotorConfigs.CurrentLimits.SupplyCurrentThreshold = 15;
        // turnMotorConfigs.CurrentLimits.SupplyTimeThreshold = 0.25;
        turnMotorConfigs.Audio.AllowMusicDurDisable = true;
        turnConfigurator.apply(turnMotorConfigs);

        refreshPID();
    }

    public void refreshPID() {
        turningController.setPID(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning);
        
        driveConfigurator.refresh(drivePIDConfigs);
        this.drivePIDConfigs.kP = ModuleConstants.kPDrive;
        this.drivePIDConfigs.kI = ModuleConstants.kIDrive;
        this.drivePIDConfigs.kD = ModuleConstants.kDDrive;
        this.drivePIDConfigs.kV = ModuleConstants.kVDrive;

        driveConfigurator.apply(drivePIDConfigs);
    }

    /**
     * Set the percent output of both motors to zero.
     */
    public void stop() {
        driveMotor.setControl(brakeRequest);
        turnMotor.setControl(brakeRequest);

        this.desiredState = new SwerveModuleState(0, Rotation2d.fromRadians(getTurningPosition()));
    }

    public void run() {
        desiredState.optimize(Rotation2d.fromRadians(getTurningPosition()));

        desiredAngle = desiredState.angle.getDegrees();

        double velocity = desiredState.speedMetersPerSecond / ModuleConstants.kMetersPerRevolution / ModuleConstants.kDriveMotorGearRatio;
        // double velocity = desiredState.speedMetersPerSecond / ModuleConstants.kDriveTicksPer100MsToMetersPerSec / ModuleConstants.kDriveMotorGearRatio;
        this.desiredVelocity = velocity;
        
    if (Math.abs(velocity) < 0.0001) {
            driveMotor.setControl(brakeRequest);
        }
        else if (this.velocityControl) {
            driveVelocityRequest.Slot = 0;
            driveMotor.setControl(driveVelocityRequest.withVelocity(velocity));
            this.currentPercent = 0;
        } else {
            this.currentPercent = desiredState.speedMetersPerSecond / SwerveDriveConstants.kPhysicalMaxSpeedMetersPerSecond;
            this.driveRequest.Output = currentPercent;
            driveMotor.setControl(this.driveRequest);
        }
        
        double turnPower = turningController.calculate(getTurningPosition(), desiredState.angle.getRadians());
        currentTurnPercent = turnPower;
        this.turnRequest.Output = currentTurnPercent;
        turnMotor.setControl(this.turnRequest);
    }

    public void resetDesiredAngle() {
        this.desiredAngle = 0;
    }
    
    //****************************** GETTERS ******************************/

    /**
     * Get the distance travelled by the motor in meters
     * @return Distance travelled by motor (in meters)
     */
    public double getDrivePosition() {
        return driveMotor.getRotorPosition().getValueAsDouble()
            * ModuleConstants.kMetersPerRevolution
            * ModuleConstants.kDriveMotorGearRatio;
    }

    public double getDrivePositionTicks() {
        return driveMotor.getRotorPosition().getValueAsDouble() * 2048;
    }


    /**
     * Get the turning motor's CANCoder's angle
     * @return Angle in radians
     */
    public double getTurningPosition() {
        double turningPosition = Math.toRadians(getTurningPositionDegrees());
        return turningPosition;
    }

    /**
     * Get the turning motor's CANCoder's angle
     * @return Angle in degrees
     */
    public double getTurningPositionDegrees() {
        double turningPosition = ((360 * canCoder.getAbsolutePosition().getValueAsDouble()) % 360 + 360) % 360;
        if (turningPosition > 180) turningPosition -= 360;
        return turningPosition;
    }

    /**
     * Get the velocity of the drive motor
     * @return Velocity of the drive motor (in meters / sec)
     */
    public double getDriveVelocity() {
        return driveMotor.getRotorVelocity().getValueAsDouble() 
            * ModuleConstants.kMetersPerRevolution
            * ModuleConstants.kDriveMotorGearRatio;
    }

    /**
     * Get the velocity of the drive motor
     * @return Velocity of the drive motor (in meters / sec)
     */
    public double getDriveVelocityRPS() {
        return driveMotor.getRotorVelocity().getValueAsDouble();
    }

    /**
     * Get the velocity of the turning motor
     * @return Velocity of the turning motor (in radians / sec)
     */
    public double getTurningVelocity() {
        double turnVelocity = Math.toRadians(getTurningVelocityDegrees());
        return turnVelocity;
    }

    /**
     * Get the velocity of the turning motor
     * @return Velocity of the turning motor (in degrees / sec)
     */
    public double getTurningVelocityDegrees() {
        double turnVelocity = canCoder.getVelocity().getValueAsDouble();
        return turnVelocity;
    }

    /**
     * Return the current state (velocity and rotation) of the Swerve Module
     * @return This Swerve Module's State
     */
    public SwerveModuleState getState() {
        currState.speedMetersPerSecond = getDriveVelocity();
        currState.angle = Rotation2d.fromRadians(getTurningPosition());
        return currState;
        // return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));

    }

    public SwerveModulePosition getPosition() {
        currPosition.distanceMeters = getDrivePosition();
        currPosition.angle = Rotation2d.fromRadians(getTurningPosition());
        return currPosition;
        //return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    //****************************** SETTERS ******************************/

    /**
     * Set the desired state of the Swerve Module and move towards it
     * @param state The desired state for this Swerve Module
     */
    public void setDesiredState(SwerveModuleState state, boolean withVelocityControl) {
        this.velocityControl = withVelocityControl;
        setDesiredState(state);
    }

    /**
     * Set the desired state of the Swerve Module and move towards it
     * @param state The desired state for this Swerve Module
     */
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            state.speedMetersPerSecond = 0;
        }

        this.desiredState = state;
    }

    public void toggleVelocityControl(boolean velocityControlOn) {
        this.velocityControl = velocityControlOn;
    }

    public void initShuffleboard(LOG_LEVEL level) {
        if (level == LOG_LEVEL.OFF)  {
            return;
        }
        int moduleId = (driveMotorID / 10);
        ShuffleboardTab tab = Shuffleboard.getTab("Module " + moduleId);

        switch (level) {
            case OFF:
                break;
            case ALL:
                tab.addNumber("Module " + moduleId + " Turn percent (motor controller)", () -> turnMotor.getDutyCycle().getValue());
                tab.addNumber("Module " + moduleId + " Turn percent (current)", () -> this.currentTurnPercent);
            case MEDIUM:
                tab.addNumber("Module " + moduleId + " Turn Motor Current", () -> turnMotor.getStatorCurrent().getValueAsDouble());
                tab.addNumber("Module " + moduleId + " Drive Motor Voltage", () -> (driveMotor.getDutyCycle().getValue() * driveMotor.getSupplyVoltage().getValueAsDouble()));
                tab.addNumber("Module " + moduleId + " Turn Motor Voltage", () -> turnMotor.getSupplyVoltage().getValueAsDouble());// ::getMotorOutputVoltage);
                tab.addNumber("Module " + moduleId + " Drive percent (motor controller)", () -> driveMotor.getDutyCycle().getValue());
                tab.addNumber("Module " + moduleId + " Drive percent (current)", () -> this.currentPercent);
                
                tab.addNumber("Module " + moduleId + " Drive ticks", this::getDrivePositionTicks);
                tab.addNumber("Module " + moduleId + " Turn angle percent", () -> turnMotor.getDutyCycle().getValue());
                tab.addNumber("Module " + moduleId + " Angle Difference", () -> desiredAngle - currentAngle);
            case MINIMAL:
                tab.addNumber("Module " + moduleId + " Turn angle", this::getTurningPositionDegrees);
                tab.addNumber("Module " + moduleId + " Desired Angle", () -> desiredAngle);
                tab.addNumber("Module " + moduleId + " Drive Supply Current", () -> driveMotor.getSupplyCurrent().getValueAsDouble());
                // tab.addNumber("Module Velocity", this::getDriveVelocity);
                tab.addNumber("Module " + moduleId + " Velocity RPS", this::getDriveVelocityRPS);
                tab.addNumber("Module " + moduleId + " Desired Velocity", () -> this.desiredVelocity);
                tab.addBoolean("Module " + moduleId + " Velocity Control", () -> this.velocityControl);
                // tab.addString("Error Status", () -> driveMotor.getFaultField().getName());
                tab.addNumber("Module " + moduleId + " Drive Stator Current", () -> driveMotor.getStatorCurrent().getValueAsDouble());
                tab.addNumber("Module " + moduleId + " Drive Temperature", () -> driveMotor.getDeviceTemp().getValueAsDouble());
                tab.addNumber("Module " + moduleId + " Turn Temperature", () -> turnMotor.getDeviceTemp().getValueAsDouble());
                break;
            }
            
    }

     public void reportToSmartDashboard(LOG_LEVEL level) {
    //     currentAngle = Math.toDegrees(getTurningPosition());
    //     switch (level) {
    //         case OFF:
    //             break;
    //         case ALL:
    //             SmartDashboard.putNumber("Drive Motor #" + driveMotorID + " Current", driveMotor.getStatorCurrent().getValue());
    //             SmartDashboard.putNumber("Turn Motor #" + turnMotorID + " Current", turnMotor.getStatorCurrent().getValue());
    //             SmartDashboard.putNumber("Drive Motor #" + driveMotorID + " Voltage", (driveMotor.getDutyCycle().getValue() * driveMotor.getSupplyVoltage().getValue()));
    //             SmartDashboard.putNumber("Turn Motor #" + turnMotorID + " Voltage", (turnMotor.getDutyCycle().getValue() * turnMotor.getSupplyVoltage().getValue()));
    //             SmartDashboard.putNumber("Turn Offset", this.CANCoderOffsetDegrees);
    //         case MEDIUM:
    //             SmartDashboard.putNumber("Module velocity #" + driveMotorID, getDriveVelocity());
    //             SmartDashboard.putNumber("Drive percent #" + driveMotorID, driveMotor.getDutyCycle().getValue());
    //             SmartDashboard.putNumber("Turn Angle #" + turnMotorID, currentAngle);
    //             SmartDashboard.putNumber("Turn Error #" + turnMotorID, desiredAngle - currentAngle);
    //         case MINIMAL:
    //             break;
    //     }

     }

    /**
     * Enable or disable the break mode on the motors
     * @param breaking  Whether or not the motor should be on break mode
     */
    public void setBreak(boolean breaking) {
        NeutralModeValue mode = (breaking ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        
        MotorOutputConfigs turnConfigs = new MotorOutputConfigs();
        this.turnConfigurator.refresh(turnConfigs);
        turnConfigs.NeutralMode = mode;
        this.turnConfigurator.apply(turnConfigs);
        
        MotorOutputConfigs driveConfigs = new MotorOutputConfigs();
        this.driveConfigurator.refresh(driveConfigs);
        driveConfigs.NeutralMode = mode;
        this.driveConfigurator.apply(driveConfigs);
    }
}