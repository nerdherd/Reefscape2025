package frc.robot.util;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controller {
    private final boolean isPS4;
    private final boolean isPS5;

    private CommandPS4Controller cmdPS4;
    private PS4Controller PS4;
    private CommandPS5Controller cmdPS5;
    private PS5Controller PS5;
    private GuliKit guliKit;

    /**
     * PS4 controller implemntation.
     * <p>Combines CommandPS4Controller and PS4Controller.
     * @param port The port of the controller on FRC Driver Station
     */
    public Controller(int port) {
        isPS4 = true;
        isPS5 = false;

        cmdPS4 = new CommandPS4Controller(port);
        PS4 = cmdPS4.getHID();
    }

    /**
     * PS4/PS5 controller implemntation.
     * <p>Combines CommandPS4Controller, PS4Controller, CommandPS5Controller, and PS5Controller. 
     * @param port The port of the controller on FRC Driver Station
     * @param usePS4 Whether to use PS4 (true) or PS5 (false)
     */
    public Controller(int port, boolean usePS4) {
        isPS4 = usePS4;
        isPS5 = !usePS4;

        if (isPS4){
            cmdPS4 = new CommandPS4Controller(port);
            PS4 = cmdPS4.getHID();
        } else {
            cmdPS5 = new CommandPS5Controller(port);
            PS5 = cmdPS5.getHID();
        }
    }

    /**
     * GuliKit controller implementation.
     * @param port The port of the controller on FRC Driver Station
     * @param isDigLeft If the left switch on the controller is set to digital (dot)
     * @param isDigRight If the right switch on the controller is set to digital (dot)
     */
    public Controller(int port, boolean isDigLeft, boolean isDigRight) {
        isPS4 = false;
        isPS5 = false;

        guliKit = new GuliKit(port, isDigLeft, isDigRight);
    }

    // ***** VALUE METHODS ***** //
    // TODO: Ensure all of these bindings are correct

    public double getLeftX() { return isPS4 ? PS4.getLeftX() : (isPS5 ? PS5.getLeftX() : guliKit.getLeftX()); }
    public double getLeftY() { return isPS4 ? PS4.getLeftY() : (isPS5 ? PS5.getLeftY() : guliKit.getLeftY()); }
    public double getRightX() { return isPS4 ? PS4.getRightX() : (isPS5 ? PS5.getRightX() :guliKit.getRightX()); }
    public double getRightY() { return isPS4 ? PS4.getRightY() : (isPS5 ? PS5.getRightY() : guliKit.getRightY()); }

    public boolean getButtonRight() { return isPS4 ? PS4.getCircleButton() : (isPS5 ? PS5.getCircleButton() : guliKit.getA()); }
    public boolean getButtonDown() { return isPS4 ? PS4.getCrossButton() : (isPS5 ? PS5.getCrossButton() : guliKit.getB()); }
    public boolean getButtonUp() { return isPS4 ? PS4.getTriangleButton() : (isPS5 ? PS5.getTriangleButton() : guliKit.getX()); }
    public boolean getButtonLeft() { return isPS4 ? PS4.getSquareButton() : (isPS5 ? PS5.getSquareButton() : guliKit.getY()); }

    public boolean getTriggerLeft() { return isPS4 ? PS4.getL2Button() : (isPS5 ? PS5.getL2Button() : guliKit.getZLdigital()); }
    public boolean getTriggerRight() { return isPS4 ? PS4.getR2Button() : (isPS5 ? PS5.getR2Button() : guliKit.getZRdigital()); }
    public boolean getBumperLeft() { return isPS4 ? PS4.getL1Button() : (isPS5 ? PS5.getL1Button() : guliKit.getL()); }
    public boolean getBumperRight() { return isPS4 ? PS4.getR1Button() : (isPS5 ? PS5.getR1Button() : guliKit.getR()); }

    public boolean getControllerLeft() { return isPS4 ? PS4.getShareButton() : (isPS5 ? PS5.getCreateButton() : guliKit.getMinus()); }
    public boolean getControllerRight() { return isPS4 ? PS4.getOptionsButton() : (isPS5 ? PS5.getOptionsButton() : guliKit.getPlus()); }
    public boolean getJoystickLeft() { return isPS4 ? PS4.getL3Button() : (isPS5 ? PS5.getL3Button() : guliKit.getLeftJoy()); }
    public boolean getJoystickRight() { return isPS4 ? PS4.getR3Button() : (isPS5 ? PS5.getR3Button() : guliKit.getRightJoy()); }

    /** <STRONG> PS4/5 ONLY </STRONG> */
    public boolean getDpadUp(EventLoop loop) { return isPS4 ? PS4.povUp(loop).getAsBoolean() : PS5.povUp(loop).getAsBoolean(); }
    /** <STRONG> GULIKIT ONLY </STRONG> */
    public boolean getDpadUp() { return guliKit.getDpadUp(); }
    /** <STRONG> PS4 ONLY </STRONG> */
    public boolean getDpadRight(EventLoop loop) { return isPS4 ? PS4.povRight(loop).getAsBoolean() : PS5.povRight(loop).getAsBoolean(); }
    /** <STRONG> GULIKIT ONLY </STRONG> */
    public boolean getDpadRight() { return guliKit.getDpadRight(); }
    /** <STRONG> PS4 ONLY </STRONG> */
    public boolean getDpadDown(EventLoop loop) { return isPS4 ? PS4.povDown(loop).getAsBoolean() : PS5.povDown(loop).getAsBoolean(); }
    /** <STRONG> GULIKIT ONLY </STRONG> */
    public boolean getDpadDown() { return guliKit.getDpadDown(); }
    /** <STRONG> PS4 ONLY </STRONG> */
    public boolean getDpadLeft(EventLoop loop) { return isPS4 ? PS4.povLeft(loop).getAsBoolean() : PS5.povLeft(loop).getAsBoolean(); }
    /** <STRONG> GULIKIT ONLY </STRONG> */
    public boolean getDpadLeft() { return guliKit.getDpadLeft(); }

    // ***** OBJECT METHODS ***** //

    public Trigger buttonRight() { return isPS4 ? cmdPS4.circle() : (isPS5 ? cmdPS5.circle() : guliKit.buttonA()); }
    public Trigger buttonDown() { return isPS4 ? cmdPS4.cross() : (isPS5 ? cmdPS5.cross() : guliKit.buttonA()); }
    public Trigger buttonUp() { return isPS4 ? cmdPS4.triangle() : (isPS5 ? cmdPS5.triangle() : guliKit.buttonA()); }
    public Trigger buttonLeft() { return isPS4 ? cmdPS4.square() : (isPS5 ? cmdPS5.square() : guliKit.buttonA()); }

    public Trigger triggerLeft() { return isPS4 ? cmdPS4.L2() : (isPS5 ? cmdPS5.L2() : guliKit.triggerZL()); }
    public Trigger triggerRight() { return isPS4 ? cmdPS4.R2() : (isPS5 ? cmdPS5.R2() : guliKit.triggerZR()); }
    public Trigger bumperLeft() { return isPS4 ? cmdPS4.L1() : (isPS5 ? cmdPS5.L1() : guliKit.bumperL()); }
    public Trigger bumperRight() { return isPS4 ? cmdPS4.R1() : (isPS5 ? cmdPS5.R1() : guliKit.bumperR()); }

    public Trigger controllerLeft() { return isPS4 ? cmdPS4.share() : (isPS5 ? cmdPS5.create() : guliKit.buttonMinus()); }
    public Trigger controllerRight() { return isPS4 ? cmdPS4.options() : (isPS5 ? cmdPS5.options() : guliKit.buttonPlus()); }
    public Trigger joystickLeft() { return isPS4 ? cmdPS4.L3() : (isPS5 ? cmdPS5.L3() : guliKit.buttonLeftJoy()); }
    public Trigger joystickRight() { return isPS4 ? cmdPS4.R3() : (isPS5 ? cmdPS5.R3() : guliKit.buttonRightJoy()); }

    public Trigger dpadUp() { return isPS4 ? cmdPS4.povUp() : (isPS5 ? cmdPS5.povUp() : guliKit.dpadUp()); }
    public Trigger dpadRight() { return isPS4 ? cmdPS4.povRight() : (isPS5 ? cmdPS5.povRight() : guliKit.dpadRight()); }
    public Trigger dpadDown() { return isPS4 ? cmdPS4.povDown() : (isPS5 ? cmdPS5.povDown() : guliKit.dpadDown()); }
    public Trigger dpadLeft() { return isPS4 ? cmdPS4.povLeft() : (isPS5 ? cmdPS5.povLeft() : guliKit.dpadLeft()); }

    // ***** STATE METHODS ***** //

    /** <STRONG> GULIKIT ONLY </STRONG> */
    public void setDigitalLeft(boolean isDigital) {
        if (!isPS4) guliKit.setDigitalLeft(isDigital);
    }
    /** <STRONG> GULIKIT ONLY </STRONG> */
    public void setDigitalRight(boolean isDigital) {
        if (!isPS4) guliKit.setDigitalRight(isDigital);
    }

    /** <STRONG> GULIKIT ONLY </STRONG> */
    public boolean isDigLeft() {
        if (!isPS4) return guliKit.isDigLeft();
        else return true;
    }
    /** <STRONG> GULIKIT ONLY </STRONG> */
    public boolean isDigRight() {
        if (!isPS4) return guliKit.isDigRight();
        else return true;
    }
}
