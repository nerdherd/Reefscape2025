// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sims;

import java.util.ArrayList;

import frc.robot.sims.simulations.GenericSimulation;

/** Add your docs here. */
public class PhysicsSimulator {
    private final static PhysicsSimulator instance = new PhysicsSimulator();

    public static PhysicsSimulator getInstance() { return instance; }

    private final ArrayList<GenericSimulation> _sims = new ArrayList<GenericSimulation>();
    public void addSimulation(GenericSimulation sim) {
        _sims.add(sim);
    }

    public void run() {
        for (GenericSimulation sim : _sims) {
            sim.run();
        }
    }
}
