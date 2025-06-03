// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sims.simulations;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/** Add your docs here. */
public class DynamicElevatorSim extends ElevatorSim {
    // The min allowable height for the elevator.
    private final double m_minHeight;

    // The max allowable height for the elevator.
    private final double m_maxHeight;

    // Whether the simulator should simulate gravity.
    private final boolean m_simulateGravity;

    private final DoubleSupplier m_angleSupplier;

    /**
     * Creates a simulated elevator mechanism.
     *
     * @param plant The linear system that represents the elevator. This system can be created with
     *     {@link edu.wpi.first.math.system.plant.LinearSystemId#createElevatorSystem(DCMotor, double,
     *     double, double)}.
     * @param gearbox The type of and number of motors in the elevator gearbox.
     * @param minHeightMeters The min allowable height of the elevator.
     * @param maxHeightMeters The max allowable height of the elevator.
     * @param simulateGravity Whether gravity should be simulated or not.
     * @param startingHeightMeters The starting height of the elevator.
     * @param measurementStdDevs The standard deviations of the measurements. Can be omitted if no
     *     noise is desired. If present must have 1 element for position.
     */
    @SuppressWarnings("this-escape")
    public DynamicElevatorSim(
    LinearSystem<N2, N1, N2> plant,
    DCMotor gearbox,
    DoubleSupplier angleSupplier,
    double minHeightMeters,
    double maxHeightMeters,
    boolean simulateGravity,
    double startingHeightMeters,
    double... measurementStdDevs) {
    super(plant, gearbox, minHeightMeters, maxHeightMeters, simulateGravity, startingHeightMeters, measurementStdDevs);
    this.m_minHeight = minHeightMeters;
    this.m_maxHeight = maxHeightMeters;
    this.m_simulateGravity = simulateGravity;
    this.m_angleSupplier = angleSupplier;
    }

    /**
    * Creates a simulated elevator mechanism.
    *
    * @param kV The velocity gain.
    * @param kA The acceleration gain.
    * @param gearbox The type of and number of motors in the elevator gearbox.
    * @param minHeightMeters The min allowable height of the elevator.
    * @param maxHeightMeters The max allowable height of the elevator.
    * @param simulateGravity Whether gravity should be simulated or not.
    * @param startingHeightMeters The starting height of the elevator.
    * @param measurementStdDevs The standard deviations of the measurements. Can be omitted if no
    *     noise is desired. If present must have 1 element for position.
    */
    public DynamicElevatorSim(
        double kV,
        double kA,
        DCMotor gearbox,
        DoubleSupplier angleSupplier,
        double minHeightMeters,
        double maxHeightMeters,
        boolean simulateGravity,
        double startingHeightMeters,
        double... measurementStdDevs) {
        super(kV, kA, gearbox, minHeightMeters, maxHeightMeters, simulateGravity, startingHeightMeters, measurementStdDevs);
        this.m_minHeight = minHeightMeters;
        this.m_maxHeight = maxHeightMeters;
        this.m_simulateGravity = simulateGravity;
        this.m_angleSupplier = angleSupplier;
    }

    /**
     * Creates a simulated elevator mechanism.
     *
     * @param gearbox The type of and number of motors in the elevator gearbox.
     * @param gearing The gearing of the elevator (numbers greater than 1 represent reductions).
     * @param carriageMassKg The mass of the elevator carriage.
     * @param drumRadiusMeters The radius of the drum that the elevator spool is wrapped around.
     * @param minHeightMeters The min allowable height of the elevator.
     * @param maxHeightMeters The max allowable height of the elevator.
     * @param simulateGravity Whether gravity should be simulated or not.
     * @param startingHeightMeters The starting height of the elevator.
     * @param measurementStdDevs The standard deviations of the measurements. Can be omitted if no
     *     noise is desired. If present must have 1 element for position.
     */
    public DynamicElevatorSim(
        DCMotor gearbox,
        DoubleSupplier angleSupplier,
        double gearing,
        double carriageMassKg,
        double drumRadiusMeters,
        double minHeightMeters,
        double maxHeightMeters,
        boolean simulateGravity,
        double startingHeightMeters,
        double... measurementStdDevs) {
        super(gearbox, gearing, carriageMassKg, drumRadiusMeters, minHeightMeters, maxHeightMeters, simulateGravity, startingHeightMeters, measurementStdDevs);
        this.m_minHeight = minHeightMeters;
        this.m_maxHeight = maxHeightMeters;
        this.m_simulateGravity = simulateGravity;
        this.m_angleSupplier = angleSupplier;
    }

    /**
     * Updates the state of the elevator.
     *
     * @param currentXhat The current state estimate.
     * @param u The system inputs (voltage).
     * @param dtSeconds The time difference between controller updates.
     */
    @Override
    protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
        // Calculate updated x-hat from Runge-Kutta.
        var updatedXhat =
            NumericalIntegration.rkdp(
                (x, _u) -> {
                Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(_u));
                if (m_simulateGravity) {
                    xdot = xdot.plus(VecBuilder.fill(0, -9.8 * Math.sin(m_angleSupplier.getAsDouble())));
                }
                return xdot;
                },
                currentXhat,
                u,
                dtSeconds);

        // We check for collisions after updating x-hat.
        if (wouldHitLowerLimit(updatedXhat.get(0, 0))) {
        return VecBuilder.fill(m_minHeight, 0);
        }
        if (wouldHitUpperLimit(updatedXhat.get(0, 0))) {
        return VecBuilder.fill(m_maxHeight, 0);
        }
        return updatedXhat;
    }
}
