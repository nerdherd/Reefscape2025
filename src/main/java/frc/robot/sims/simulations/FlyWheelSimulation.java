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
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class FlywheelSimulation extends GenericSimulation {
    private final TalonFXSimState _talonFXSim;

    private final DCMotorSim _motorSim;

    public FlywheelSimulation(final TalonFX talonFX, final double rotorInertia) {
        super();
        this._talonFXSim = talonFX.getSimState();
        TalonFXConfiguration config = new TalonFXConfiguration();
        talonFX.getConfigurator().refresh(config);
        _talonFXSim.Orientation = (config.MotorOutput.Inverted == InvertedValue.Clockwise_Positive) ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
        var gearbox = DCMotor.getKrakenX60Foc(1);
        this._motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, rotorInertia, 1.0), gearbox);
    }

    public void run() {
        _talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        _motorSim.setInputVoltage(_talonFXSim.getMotorVoltage());
        _motorSim.update(getPeriod());

        final double position_rot = _motorSim.getAngularPositionRotations();
        final double velocity_rps = Units.radiansToRotations(_motorSim.getAngularVelocityRadPerSec());

        _talonFXSim.setRawRotorPosition(position_rot);
        _talonFXSim.setRotorVelocity(velocity_rps);
    }
}
