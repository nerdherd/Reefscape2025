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
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

/** Add your docs here. */
public class ArmSimulation extends GenericSimulation {
    private final TalonFXSimState _talonFXSim;

    private final SingleJointedArmSim _motorSim;
    
    private final MechanismLigament2d _ligament;
    /**
     * 
     * @param talonFX
     * @param gearing - >1 mean reductions
     * @param momentOfInertia - jKgm/s
     * @param armLength - meters
     * @param minAngle - radians
     * @param maxAngle - radians
     * @param startingAngle - radians
     */
    public ArmSimulation(final TalonFX talonFX, final MechanismLigament2d ligament, final double gearing, final double momentOfInertia, final double armLength, final double minAngle, final double maxAngle, final double startingAngle) {
        super();
        this._talonFXSim = talonFX.getSimState();
        this._ligament = ligament;
        TalonFXConfiguration config = new TalonFXConfiguration();
        talonFX.getConfigurator().refresh(config);
        _talonFXSim.Orientation = (config.MotorOutput.Inverted == InvertedValue.Clockwise_Positive) ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
        var gearbox = DCMotor.getKrakenX60Foc(1);
        this._motorSim = new SingleJointedArmSim(gearbox, gearing, momentOfInertia, armLength, minAngle, maxAngle, true, startingAngle);
    }

    public void run() {
        _talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        _motorSim.setInputVoltage(_talonFXSim.getMotorVoltage());
        _motorSim.update(getPeriod());
        final double position_rot = Units.radiansToRotations(_motorSim.getAngleRads());
        final double velocity_rps = Units.radiansToRotations(_motorSim.getVelocityRadPerSec());
        _talonFXSim.setRawRotorPosition(position_rot);
        _talonFXSim.setRotorVelocity(velocity_rps);

        _ligament.setAngle(Units.radiansToDegrees(_motorSim.getAngleRads()));
    }
}
