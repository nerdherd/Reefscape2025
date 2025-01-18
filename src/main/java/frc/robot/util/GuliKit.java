package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class GuliKit {
    private final Joystick controller;
    private final JoystickButton buttonB, buttonA, buttonY, buttonZ,
                                 bumperL, bumperR,
                                 buttonMinus, buttonPlus,
                                 buttonLJ, buttonRJ;

    public GuliKit(int port) {
        controller = new Joystick(port);
        buttonB = new JoystickButton(controller, 1);
        buttonA = new JoystickButton(controller, 2);
        buttonY = new JoystickButton(controller, 3);
        buttonZ = new JoystickButton(controller, 4);
        bumperL = new JoystickButton(controller, 5);
        bumperR = new JoystickButton(controller, 6);
        buttonMinus = new JoystickButton(controller, 7);
        buttonPlus = new JoystickButton(controller, 8);
        buttonLJ = new JoystickButton(controller, 9);
        buttonRJ = new JoystickButton(controller, 10);
    }

    public JoystickButton getButtonB() {return buttonB;}
    public JoystickButton getButtonA() {return buttonA;}
    public JoystickButton getButtonY() {return buttonY;} 
    public JoystickButton getButtonZ() {return buttonZ;}
    public JoystickButton getBumperL() {return bumperL;}
    public JoystickButton getBumperR() {return bumperR;}
    public JoystickButton getButtonMinus() {return buttonMinus;}
    public JoystickButton getButtonPlus() {return buttonPlus;}
    public JoystickButton getButtonLJ() {return buttonLJ;}
    public JoystickButton getButtonRJ() {return buttonRJ;}

    public double getLeftX() { return controller.getRawAxis(0); }
    public double getLeftY() { return controller.getRawAxis(1); }
    public double getTriggerZL() { return controller.getRawAxis(2); }
    public double getTriggerZR() { return controller.getRawAxis(3); }
    public double getRightX() { return controller.getRawAxis(4); }
    public double getRightY() { return controller.getRawAxis(5); }

    public int getDpadUp() { return controller.getPOV(0); }
    public int getDpadRight() { return controller.getPOV(90); }
    public int getDpadDown() { return controller.getPOV(180); }
    public int getDpadLeft() { return controller.getPOV(270); }
}
