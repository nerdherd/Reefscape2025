package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class GuliKit {
    private final Joystick controller;
    private final JoystickButton buttonB, buttonA, buttonY, buttonX,
                                 bumperL, bumperR,
                                 buttonMinus, buttonPlus,
                                 buttonLeftJoy, buttonRightJoy;
    
    private boolean isDigLeft;
    private boolean isDigRight;

    /**
     * Creates an instance of a GuliKit controller.
     * @param port The port of the controller on FRC Driver Station
     * @param isDigLeft If the left switch on the controller is set to digital (dot)
     * @param isDigRight If the right switch on the controller is set to digital (dot)
     */
    public GuliKit(int port, boolean isDigLeft, boolean isDigRight) {
        this.isDigLeft = isDigLeft;
        this.isDigRight = isDigRight;

        controller = new Joystick(port);

        buttonB = new JoystickButton(controller, 1);
        buttonA = new JoystickButton(controller, 2);
        buttonY = new JoystickButton(controller, 3);
        buttonX = new JoystickButton(controller, 4);
        bumperL = new JoystickButton(controller, 5);
        bumperR = new JoystickButton(controller, 6);
        buttonMinus    = new JoystickButton(controller, 7);
        buttonPlus     = new JoystickButton(controller, 8);
        buttonLeftJoy  = new JoystickButton(controller, 9);
        buttonRightJoy = new JoystickButton(controller, 10);
    }

    // STATE METHODS \\
    public void setDigitalLeft(boolean isDigital) { this.isDigLeft = isDigital; }
    public void setDigitalRight(boolean isDigital) { this.isDigRight = isDigital; }

    public boolean isDigLeft() { return isDigLeft; }
    public boolean isDigRight() { return isDigRight; }

    // VALUE METHODS \\
    public double getLeftX() { return controller.getRawAxis(0); }
    public double getLeftY() { return controller.getRawAxis(1); }
    public double getRightX() { return controller.getRawAxis(4); }
    public double getRightY() { return controller.getRawAxis(5); }

    public boolean getB() { return buttonB.getAsBoolean(); }
    public boolean getA() { return buttonA.getAsBoolean(); }
    public boolean getY() { return buttonX.getAsBoolean(); } 
    public boolean getX() { return buttonY.getAsBoolean(); }
    public boolean getL() { return bumperL.getAsBoolean(); }
    public boolean getR() { return bumperR.getAsBoolean(); }
    public boolean getZLdigital() { return controller.getRawAxis(2) > 0.65 ? true : false; }
    public boolean getZRdigital() { return controller.getRawAxis(3) > 0.65 ? true : false; }
    public double getZLanalog() { return controller.getRawAxis(2); }
    public double getZRanalog() { return controller.getRawAxis(3); }

    public boolean getMinus() { return buttonMinus.getAsBoolean(); }
    public boolean getPlus() { return buttonPlus.getAsBoolean(); }
    public boolean getLeftJoy() { return buttonLeftJoy.getAsBoolean(); }
    public boolean getRightJoy() { return buttonRightJoy.getAsBoolean(); }

    public boolean getDpadUp() { return (controller.getPOV() >= 300 || controller.getPOV() <= 60) && controller.getPOV() != -1; }
    public boolean getDpadRight() { return controller.getPOV() >= 30 && controller.getPOV() <= 150; }
    public boolean getDpadDown() { return controller.getPOV() >= 120 && controller.getPOV() <= 240; }
    public boolean getDpadLeft() { return controller.getPOV() >= 210 && controller.getPOV() <= 330; }

    // OBJECT METHODS \\
    public JoystickButton buttonB() { return buttonB; }
    public JoystickButton buttonA() { return buttonA; }
    public JoystickButton buttonY() { return buttonY; } 
    public JoystickButton buttonX() { return buttonX; }
    public JoystickButton bumperL() { return bumperL; }
    public JoystickButton bumperR() { return bumperR; }
    public Trigger triggerZL() { return new Trigger(() -> controller.getRawAxis(2) > 0.65); }
    public Trigger triggerZR() { return new Trigger(() -> controller.getRawAxis(3) > 0.65); }

    public JoystickButton buttonMinus() { return buttonMinus; }
    public JoystickButton buttonPlus() { return buttonPlus; }
    public JoystickButton buttonLeftJoy() { return buttonLeftJoy; }
    public JoystickButton buttonRightJoy() { return buttonRightJoy; }

    public Trigger dpadUp() { return new Trigger(this::getDpadUp); }
    public Trigger dpadRight() { return new Trigger(this::getDpadRight); }
    public Trigger dpadDown() { return new Trigger(this::getDpadDown); }
    public Trigger dpadLeft() { return new Trigger(this::getDpadLeft); }
}
