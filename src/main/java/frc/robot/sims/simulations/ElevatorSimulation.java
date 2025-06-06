// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sims.simulations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

/** Add your docs here. */
public class ElevatorSimulation extends GenericSimulation {
    private final TalonFXSimState _talonFXSim;

    private final DynamicElevatorSim _motorSim;
    
    private final MechanismLigament2d _ligament;
    /**
     * 
     * @param talonFX
     * @param gearing - >1 mean reductions
     * @param weight - kilograms
     * @param drumRadius - meters
     * @param minHeight - meters
     * @param maxHeight - meters
     * @param startingHeight - meters
     */
    public ElevatorSimulation(final TalonFX talonFX, final MechanismLigament2d ligament, final double gearing, final double weight, final double drumRadius, final double minHeight, final double maxHeight, final double startingHeight) {
        super();
        this._talonFXSim = talonFX.getSimState();
        this._ligament = ligament;
        TalonFXConfiguration config = new TalonFXConfiguration();
        talonFX.getConfigurator().refresh(config);
        _talonFXSim.Orientation = (config.MotorOutput.Inverted == InvertedValue.Clockwise_Positive) ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
        var gearbox = DCMotor.getKrakenX60Foc(1);
        this._motorSim = new DynamicElevatorSim(gearbox, () -> Units.degreesToRadians(ligament.getAngle()), gearing, weight, drumRadius, minHeight, maxHeight, true, startingHeight);
    }

    public void run() {
        _talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        _motorSim.setInputVoltage(_talonFXSim.getMotorVoltage());
        _motorSim.update(getPeriod());
        
        final double position_rot = _motorSim.getPositionMeters(); // TODO convert from meters to rotations :((
        final double velocity_rps = _motorSim.getVelocityMetersPerSecond();
        _talonFXSim.setRawRotorPosition(position_rot);
        _talonFXSim.setRotorVelocity(velocity_rps);

        _ligament.setLength(_motorSim.getPositionMeters());
    }
}
