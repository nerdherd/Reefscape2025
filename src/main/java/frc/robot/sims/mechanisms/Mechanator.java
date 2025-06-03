// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sims.mechanisms;

import java.util.HashMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Mechanator {
    private final static Mechanator instance = new Mechanator();

    public static Mechanator getInstance() { return instance; }

    private final Mechanism2d m_mech;

    private final HashMap<Pair<Double, Double>, MechanismRoot2d> roots = new HashMap<Pair<Double, Double>, MechanismRoot2d>();
    private final HashMap<String, MechanismLigament2d> ligaments = new HashMap<String, MechanismLigament2d>();

    private Mechanator() {
        m_mech = new Mechanism2d(3, 3);
        SmartDashboard.putData("ROBOT MECHANISM", m_mech);
    }

    public MechanismLigament2d getLigament(String name, double x, double y, double length, double angle) {
        if (ligaments.containsKey(name)) {
            return ligaments.get(name);
        }
        MechanismRoot2d root;
        if (roots.containsKey(Pair.of(x, y))) {
            root = roots.get(Pair.of(x, y));
        } else { 
            root = m_mech.getRoot(name, x, y);
            roots.put(Pair.of(x, y), root); 
        }
        MechanismLigament2d ligament = root.append(new MechanismLigament2d(name, length, angle));
        ligaments.put(name, ligament);
        return ligament;
    }
}
