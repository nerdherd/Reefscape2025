package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class GuliKit {
    private final Joystick controller;
    private final JoystickButton buttonB, buttonA, buttonY, buttonX,
                                 bumperZL, bumperZR,
                                 buttonMinus, buttonPlus,
                                 buttonLeftJoy, buttonRightJoy;
    
    private boolean isDigitalLeft;
    private boolean isDigitalRight;

    public GuliKit(int port, boolean isDigitalLeft, boolean isDigitalRight) {
        this.isDigitalLeft = isDigitalLeft;
        this.isDigitalRight = isDigitalRight;

        controller = new Joystick(port);
        buttonB = new JoystickButton(controller, 1);
        buttonA = new JoystickButton(controller, 2);
        buttonX = new JoystickButton(controller, 3);
        buttonY = new JoystickButton(controller, 4);
        bumperZL = new JoystickButton(controller, 5);
        bumperZR = new JoystickButton(controller, 6);
        buttonMinus = new JoystickButton(controller, 7);
        buttonPlus = new JoystickButton(controller, 8);
        buttonLeftJoy = new JoystickButton(controller, 9);
        buttonRightJoy = new JoystickButton(controller, 10);
    }

    public void setDigitalLeft(boolean isDigital) { this.isDigitalLeft = isDigital; }
    public void setDigitalRight(boolean isDigital) { this.isDigitalRight = isDigital; }

    public boolean isDigitalLeft() { return isDigitalLeft; }
    public boolean isDigitaRight() { return isDigitalRight; }

    public JoystickButton buttonB() { return buttonB; }
    public JoystickButton buttonA() { return buttonA; }
    public JoystickButton buttonY() { return buttonY; } 
    public JoystickButton buttonX() { return buttonX; }
    public JoystickButton triggerZL() { return bumperZL; }
    public JoystickButton triggerZR() { return bumperZR; }
    public JoystickButton buttonMinus() { return buttonMinus; }
    public JoystickButton buttonPlus() { return buttonPlus; }
    public JoystickButton buttonLeftJoy() { return buttonLeftJoy; }
    public JoystickButton buttonRightJoy() { return buttonRightJoy; }

    public boolean getB() { return buttonB.getAsBoolean(); }
    public boolean getA() { return buttonA.getAsBoolean(); }
    public boolean getY() { return buttonY.getAsBoolean(); } 
    public boolean getX() { return buttonX.getAsBoolean(); }
    public boolean getZL() { return bumperZL.getAsBoolean(); }
    public boolean getZR() { return bumperZR.getAsBoolean(); }
    public boolean getMinus() { return buttonMinus.getAsBoolean(); }
    public boolean getPlus() { return buttonPlus.getAsBoolean(); }
    public boolean getLeftJoy() { return buttonLeftJoy.getAsBoolean(); }
    public boolean getRightJoy() { return buttonRightJoy.getAsBoolean(); }

    public Trigger triggerLT() { return new Trigger(() -> controller.getRawAxis(2) > 0.65); }
    public Trigger triggerRT() { return new Trigger(() -> controller.getRawAxis(3) > 0.65); }

    public double getLeftX() { return controller.getRawAxis(0); }
    public double getLeftY() { return controller.getRawAxis(1); }
    public double getLTanalog() { return controller.getRawAxis(2); }
    public boolean getLTdigital() { return controller.getRawAxis(2) == 1 ? true : false; }
    public double getRTanalog() { return controller.getRawAxis(3); }
    public boolean getRTdigital() { return controller.getRawAxis(3) == 1 ? true : false; }
    public double getRightX() { return controller.getRawAxis(4); }
    public double getRightY() { return controller.getRawAxis(5); }

    public boolean getDpadUp() { return (controller.getPOV() >= 300 || controller.getPOV() <= 60) && controller.getPOV() != 1; }
    public boolean getDpadRight() { return controller.getPOV() >= 30 && controller.getPOV() <= 150; }
    public boolean getDpadDown() { return controller.getPOV() >= 120 && controller.getPOV() <= 240; }
    public boolean getDpadLeft() { return controller.getPOV() >= 210 && controller.getPOV() <= 330; }
}
