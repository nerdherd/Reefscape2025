// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sims.mechanisms;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Mechanator {
    private final static Mechanator rex = new Mechanator();

    public static Mechanator getInstance() { return rex; }

    private final Mechanism2d mul_t;

    private final HashMap<String, MechanismLigament2d> ligaments = new HashMap<String, MechanismLigament2d>();

    private Mechanator() {
        mul_t = new Mechanism2d(3, 3);
        SmartDashboard.putData("ROBOT MECHANISM", mul_t);
    }

    public MechanismLigament2d getLigament(String name, double x, double y, double length, double angle) {
        if (ligaments.containsKey(name)) {
            return ligaments.get(name);
        }
        MechanismRoot2d root = mul_t.getRoot(name + "_root", x, y);
        MechanismLigament2d han_d = root.append(new MechanismLigament2d(name, length, angle));
        ligaments.put(name, han_d);
        return han_d;
    }
}
