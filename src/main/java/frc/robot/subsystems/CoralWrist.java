// package frc.robot.subsystems;

// import java.util.function.Supplier;

// import com.ctre.phoenix6.StatusCode;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.configs.TalonFXConfigurator;
// import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.CoralConstants;
// import frc.robot.subsystems.Reportable.LOG_LEVEL;
// import frc.robot.Constants.ControllerConstants;
// import frc.robot.util.NerdyMath;
// import frc.robot.util.filters.ExponentialSmoothingFilter;

// public class CoralWrist extends SubsystemBase{
//     private final TalonFX motor;
//     private final TalonFXConfigurator motorConfigurator;

//     private int targetTicks;
//     private Supplier<Double> manualInput;

//     private boolean enabled = true;
//     private boolean motionMagicOn = true;

//     private final ExponentialSmoothingFilter joystickFilter = new ExponentialSmoothingFilter(CoralConstants.kLowPassAlpha);
//     private final MotionMagicVoltage motionRequest = new MotionMagicVoltage(0);
//     private final DutyCycleOut dutyRequest = new DutyCycleOut(0);

//     public CoralWrist(Supplier<Double> manualInput){
//         motor = new TalonFX(CoralConstants.kWristMotorID);
//         motorConfigurator = motor.getConfigurator();

//         this.manualInput = manualInput;
//         targetTicks = (int) CoralConstants.kWristStowPosition.get();

//         // configure motor
//         TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
//         motorConfigurator.refresh(motorConfigs);

//         motorConfigs.CurrentLimits.SupplyCurrentLimit = 25;
//         motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
//         motorConfigs.CurrentLimits.SupplyCurrentLowerLimit = 30;
//         motorConfigs.CurrentLimits.SupplyCurrentLowerTime = 0.1;
//         motorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

//         CoralConstants.kPWristMotor.loadPreferences();
//         CoralConstants.kIWristMotor.loadPreferences();
//         CoralConstants.kDWristMotor.loadPreferences();
//         CoralConstants.kVWristMotor.loadPreferences();

//         motorConfigs.Slot0.kP = CoralConstants.kPWristMotor.get();
//         motorConfigs.Slot0.kI = CoralConstants.kIWristMotor.get();
//         motorConfigs.Slot0.kD = CoralConstants.kDWristMotor.get();
//         motorConfigs.Slot0.kV = CoralConstants.kVWristMotor.get();

//         StatusCode response = motorConfigurator.apply(motorConfigs);
//         if (!response.isOK()){
//             DriverStation.reportError("Could not apply motor configs, error code:" + response.toString(), new Error().getStackTrace());
//         }

//         motor.setNeutralMode(NeutralModeValue.Brake);
//     }

//     public void zeroEncodersStow() {
//         // leftEncoder.setSelectedSensorPosition(Coral WristConstants.kCoral WristStowPowerOff / Coral WristConstants.kFalconTicksPerAbsoluteTicks, 1, 1000);
//         // leftEncoder.setSelectedSensorPosition(Coral WristConstants.kCoral WristStowPowerOff / Coral WristConstants.kFalconTicksPerAbsoluteTicks, 0, 1000);
//         // pivot.setSelectedSensorPosition(Coral WristConstants.kCoral WristStowPowerOff, 0, 1000);
//         motor.setPosition(0);
//     }

//     @Override
//     public void periodic() {
//         if (enabled) {
//             if (motionMagicOn){
//                 moveMotionMagic();
//             } else {
//                 if (Math.abs(manualInput.get()) > ControllerConstants.kDeadband) {
//                     motor.setControl(dutyRequest.withOutput(0.5 * manualInput.get()));
//                 } else {
//                     motor.setControl(dutyRequest.withOutput(0));
//                 }
//             }
//         } else {
//             resetToStowPosition();
//         }
//     }

//     //****************************** POSITION METHODS ******************************//

//     public void toggleMotionMagic(boolean motionMagicOn) {
//         this.motionMagicOn = motionMagicOn;
//     }

//     public void setTargetTicks(int targetTicks) {
//         this.motionMagicOn = true;
//         this.targetTicks = targetTicks;
//     }

//     private int getPositionTicks() {
//         return (int) motor.getRotorPosition().getValueAsDouble();
//     }

//     public double getAngleDegrees() {
//         return getPositionTicks() * CoralConstants.kDegreesPerTick % 360;
//     }

//     // Use during a match to re-enable motion magic and turn off joystick input
//     public void resetToStowPosition() {
//         toggleMotionMagic(true);
//         zeroEncodersStow();
//         setTargetTicks((int) CoralConstants.kWristStowPosition.get());
//     }

//     public void moveMotionMagic(){
//         motor.setControl(motionRequest.withPosition(targetTicks));
//     }

//     public void moveJoystick(double joystickInput) {
//         if (Math.abs(joystickInput) > 0.1) {
//             int tickChange = (int) (CoralConstants.kJoystickScale * joystickInput);
//             int currentTicks = getPositionTicks();

//             // tickChange = (int) joystickFilter.calculate(tickChange);

//             targetTicks = currentTicks + tickChange;
//             targetTicks = (int) NerdyMath.clamp(targetTicks, CoralConstants.kWristLowerLimit, CoralConstants.kWristUpperLimit);
//         }
//         else {
//             joystickFilter.calculate(0);
//         }

//         setTargetTicks(targetTicks);
//     }

//         //****************************** COMMAND METHODS ******************************//

//     public Command setEnabledCommand(boolean enabled) {
//         return Commands.runOnce(() -> {this.enabled = enabled;});
//     }

//     public Command incrementTargetTicks(int increment) {
//         return Commands.runOnce(() -> {targetTicks += increment;});
//     }

//     public Command pivotTo(int ticks) {
//         return Commands.runOnce(() -> setTargetTicks(ticks));
//     }

//     public Command motionMagicCommand(int position) {
//         return Commands.runOnce(() -> {
//             TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
//             motorConfigurator.refresh(motorConfigs);
//             motorConfigs.MotionMagic.MotionMagicCruiseVelocity = CoralConstants.kWristCruiseVelocity.get();
//             motorConfigs.MotionMagic.MotionMagicAcceleration = CoralConstants.kWristCruiseAcceleration.get();
//             // StatusCode pivotResponse = pivotConfigurator.apply(pivotConfig);

//             setTargetTicks(position);
//         });
//     }

//     //****************************** NAMED COMMANDS ******************************//

//     public Command pivotToReefL1() {
//         return pivotTo((int) CoralConstants.kWristL1Position.get());
//     }

//     public Command pivotToReefL23() {
//         return pivotTo((int) CoralConstants.kWristL23Position.get());
//     }

//     public Command pivotToReefL4() {
//         return pivotTo((int) CoralConstants.kWristL4Position.get());
//     }

//     //****************************** LOGGING METHODS ******************************//

//     // TODO why can't this be an override
//     public void reportToSmartDashboard(LOG_LEVEL level) {
//         switch (level) {
//             case OFF:
//                 break;
//             case ALL:
//                 // SmartDashboard.putNumber("Coral Wrist Motor Output", pivot.getMotorOutputPercent());
//                 SmartDashboard.putNumber("Coral Wrist Angle", Math.toDegrees(getAngleDegrees()));
//                 // SmartDashboard.putNumber("Coral Wrist Velocity", pivot.getSelectedSensorVelocity());
//             case MEDIUM:
//                 SmartDashboard.putNumber("Coral Wrist Current", motor.getStatorCurrent().getValueAsDouble());
//                 // SmartDashboard.putNumber("Coral Wrist Voltage", pivot.getMotorOutputVoltage());
//             case MINIMAL:
//                 SmartDashboard.putNumber("Coral Wrist Ticks", getPositionTicks());
//                 SmartDashboard.putNumber("Target Coral Wrist Ticks", targetTicks);
//                 break;
//         }
//     }

//     public void initShuffleboard(LOG_LEVEL level) { 
//         if (level == LOG_LEVEL.OFF || level == LOG_LEVEL.MINIMAL) {
//             return;
//         }
//         ShuffleboardTab tab = Shuffleboard.getTab("Coral Wrist");
//         switch (level) {
//             case OFF:
//                 break;
//             case ALL:
//                 // tab.addNumber("Motor Output", pivot::getMotorOutputPercent);
//                 tab.addString("Control Mode", motor.getControlMode()::toString);
//                 // tab.add("Zero pivot angle", Commands.runOnce(() -> {
//                 //     // leftEncoder.setSelectedSensorPosition(0, 1, 1000);
//                 //     // leftEncoder.setSelectedSensorPosition(0, 0, 1000);
//                 //     pivot.setSelectedSensorPosition(0, 0, 1000);
//                 //     resetEncoders();
//                 // }));
//                 // tab.addNumber("Coral Wrist Target Velocity", pivot::getActiveTrajectoryVelocity); 
//                 // tab.addNumber("Closed loop error", pivot::getClosedLoopError);
//                 // tab.add("Stow pivot angle", Commands.runOnce(() -> {
//                 //     // leftEncoder.setSelectedSensorPosition(Coral WristConstants.kCoral WristStowPowerOff / Coral WristConstants.kFalconTicksPerAbsoluteTicks, 1, 1000);
//                 //     // leftEncoder.setSelectedSensorPosition(Coral WristConstants.kCoral WristStowPowerOff / Coral WristConstants.kFalconTicksPerAbsoluteTicks, 0, 1000);
//                 //     pivot.setSelectedSensorPosition(Coral WristConstants.kCoral WristStowPowerOff, 0, 1000);
//                 // }));

//             case MEDIUM:
//                 // tab.addNumber("Coral Wrist Stator Current", pivot::getStatorCurrent);
//                 // tab.addNumber("Coral Wrist Supply Curremt", pivot::getSupplyCurrent);
//                 // tab.addNumber("Coral Wrist Velocity", pivot::getSelectedSensorVelocity);
//                 // tab.addNumber("Coral Wrist Voltage", pivot::getMotorOutputVoltage);
//                 // tab.addNumber("Coral Wrist Percent Output", pivot::getMotorOutputPercent);

//             case MINIMAL:
//                 tab.addNumber("Current Coral Wrist Ticks", this::getPositionTicks);
//                 // tab.addNumber("Current Coral Wrist Absolute Ticks", () -> leftEncoder.getSelectedSensorPosition(0));
//                 // tab.addNumber("Current Coral Wrist Absolute Angle", () -> leftEncoder.getSelectedSensorPosition(0) / 4096 * 360);
//                 tab.addNumber("Target Coral Wrist Ticks", () -> targetTicks);
//                 // tab.addNumber("MotionMagic Velocity", pivot::getActiveTrajectoryVelocity);
//                 // tab.addNumber("MotionMagic Position", pivot::getActiveTrajectoryPosition);
//                 tab.addNumber("Current Coral Wrist Angle", this::getAngleDegrees);
//                 break;
//         }
//     }
// }