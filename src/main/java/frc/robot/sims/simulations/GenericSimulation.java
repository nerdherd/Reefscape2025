// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sims.simulations;

import com.ctre.phoenix6.Utils;

import frc.robot.sims.PhysicsSimulator;

public class GenericSimulation {
    private double _lastTime;
    private boolean _running = false;
    
    public GenericSimulation() {
        PhysicsSimulator.getInstance().addSimulation(this);
    }

    protected double getPeriod() {
        if (!_running) {
            _lastTime = Utils.getCurrentTimeSeconds();
            _running = true;
        }

        double now = Utils.getCurrentTimeSeconds();
        final double period = now - _lastTime;
        _lastTime = now;

        return period;
    }

    public void run() {}
}
