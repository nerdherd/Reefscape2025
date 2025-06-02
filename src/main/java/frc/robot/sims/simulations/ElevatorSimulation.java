// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sims.simulations;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class ElevatorSimulation extends GenericSimulation {
    private static final double kMotorResistance = 0.002; // Assume 2mOhm resistance for voltage drop calculation
    private final TalonFXSimState _talonFXSim;

    private final ElevatorSim _motorSim;
    
    private Mechanism2d mech2d = new Mechanism2d(1, 1);
    private MechanismRoot2d root = mech2d.getRoot("elevatorSim", 0.5, 0);
    private MechanismLigament2d ligament = root.append(new MechanismLigament2d("elevator", 0, 90));
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
    public ElevatorSimulation(final TalonFX talonFX, final double gearing, final double weight, final double drumRadius, final double minHeight, final double maxHeight, final double startingHeight) {
        super();
        this._talonFXSim = talonFX.getSimState();
        var gearbox = DCMotor.getKrakenX60Foc(1);
        this._motorSim = new ElevatorSim(gearbox, gearing, weight, drumRadius, minHeight, maxHeight, true, startingHeight, 0.001, 0.0);
        SmartDashboard.putData("elebator", mech2d);
    }

    public void run() {
        /// DEVICE SPEED SIMULATION
        
        _talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        final double inputVoltage = (_talonFXSim.Orientation == ChassisReference.Clockwise_Positive) ? -_talonFXSim.getMotorVoltage() : _talonFXSim.getMotorVoltage();
        _motorSim.setInputVoltage(-inputVoltage);
        double dt = getPeriod();
        _motorSim.update(dt);
        SmartDashboard.putString("akDSJFKDSFH", _talonFXSim.Orientation.toString());
        
        /// SET SIM PHYSICS INPUTS
        final double position_rot = _motorSim.getPositionMeters(); // TODO convert from meters to rotations :((
        final double velocity_rps = _motorSim.getVelocityMetersPerSecond();
        _talonFXSim.setRawRotorPosition(-position_rot);
        _talonFXSim.setRotorVelocity(-velocity_rps);
        
        ligament.setLength(_motorSim.getPositionMeters());
    }
}
