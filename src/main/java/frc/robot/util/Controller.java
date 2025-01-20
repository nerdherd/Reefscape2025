package frc.robot.util;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controller {
    private final boolean isPS4;

    private CommandPS4Controller cmdPS4;
    private PS4Controller PS4;
    private GuliKit guliKit;

    /**
     * Generic wrapper class for PS4 controllers.
     * <p>Combines CommandPS4Controller and PS4Controller.
     * @param port
     */
    public Controller(int port) {
        isPS4 = true;

        cmdPS4 = new CommandPS4Controller(port);
        PS4 = cmdPS4.getHID();
    }

    /**
     * Generic wrapper class for GuliKit controllers.
     * @param port
     * @param isDigitalLeft
     * @param isDigitalRight
     */
    public Controller(int port, boolean isDigitalLeft, boolean isDigitalRight) {
        isPS4 = false;

        guliKit = new GuliKit(port, isDigitalLeft, isDigitalRight);
    }

    // ***** VALUE METHODS ***** //

    public double getLeftX() { return isPS4 ? PS4.getLeftX() : guliKit.getLeftX(); }
    public double getLeftY() { return isPS4 ? PS4.getLeftY() : guliKit.getLeftY(); }
    public double getRightX() { return isPS4 ? PS4.getRightX() : guliKit.getRightX(); }
    public double getRightY() { return isPS4 ? PS4.getRightY() : guliKit.getRightY(); }

    public boolean getButtonRight() { return isPS4 ? PS4.getCircleButton() : guliKit.getA(); }
    public boolean getButtonDown() { return isPS4 ? PS4.getCrossButton() : guliKit.getB(); }
    public boolean getButtonUp() { return isPS4 ? PS4.getTriangleButton() : guliKit.getX(); }
    public boolean getButtonLeft() { return isPS4 ? PS4.getSquareButton() : guliKit.getY(); }

    public boolean getTriggerLeft() { return isPS4 ? PS4.getL2Button() : guliKit.getZLdigital(); }
    public boolean getTriggerRight() { return isPS4 ? PS4.getR2Button() : guliKit.getZRdigital(); }
    public boolean getBumperLeft() { return isPS4 ? PS4.getL1Button() : guliKit.getL(); }
    public boolean getBumperRight() { return isPS4 ? PS4.getR1Button() : guliKit.getR(); }

    public boolean getControllerLeft() { return isPS4 ? PS4.getShareButton() : guliKit.getMinus(); }
    public boolean getControllerRight() { return isPS4 ? PS4.getOptionsButton() : guliKit.getPlus(); }
    public boolean getJoystickLeft() { return isPS4 ? PS4.getL3Button() : guliKit.getLeftJoy(); }
    public boolean getJoystickRight() { return isPS4 ? PS4.getR3Button() : guliKit.getRightJoy(); }

    /** <STRONG> PS4 ONLY </STRONG> */
    public boolean getDpadUp(EventLoop loop) { return PS4.povUp(loop).getAsBoolean(); }
    /** <STRONG> GULIKIT ONLY </STRONG> */
    public boolean getDpadUp() { return guliKit.getDpadUp(); }
    /** <STRONG> PS4 ONLY </STRONG> */
    public boolean getDpadRight(EventLoop loop) { return PS4.povRight(loop).getAsBoolean(); }
    /** <STRONG> GULIKIT ONLY </STRONG> */
    public boolean getDpadRight() { return guliKit.getDpadRight(); }
    /** <STRONG> PS4 ONLY </STRONG> */
    public boolean getDpadDown(EventLoop loop) { return PS4.povDown(loop).getAsBoolean(); }
    /** <STRONG> GULIKIT ONLY </STRONG> */
    public boolean getDpadDown() { return guliKit.getDpadDown(); }
    /** <STRONG> PS4 ONLY </STRONG> */
    public boolean getDpadLeft(EventLoop loop) { return PS4.povLeft(loop).getAsBoolean(); }
    /** <STRONG> GULIKIT ONLY </STRONG> */
    public boolean getDpadLeft() { return guliKit.getDpadLeft(); }

    // ***** OBJECT METHODS ***** //

    public Trigger buttonRight() { return isPS4 ? cmdPS4.circle() : guliKit.buttonA(); }
    public Trigger buttonDown() { return isPS4 ? cmdPS4.cross() : guliKit.buttonA(); }
    public Trigger buttonUp() { return isPS4 ? cmdPS4.triangle() : guliKit.buttonA(); }
    public Trigger buttonLeft() { return isPS4 ? cmdPS4.square() : guliKit.buttonA(); }

    public Trigger triggerLeft() { return isPS4 ? cmdPS4.L2() : guliKit.triggerZL(); }
    public Trigger triggerRight() { return isPS4 ? cmdPS4.R2() : guliKit.triggerZR(); }
    public Trigger bumperLeft() { return isPS4 ? cmdPS4.L1() : guliKit.bumperL(); }
    public Trigger bumperRight() { return isPS4 ? cmdPS4.R1() : guliKit.bumperR(); }

    public Trigger controllerLeft() { return isPS4 ? cmdPS4.share() : guliKit.buttonMinus(); }
    public Trigger controllerRight() { return isPS4 ? cmdPS4.options() : guliKit.buttonPlus(); }
    public Trigger joystickLeft() { return isPS4 ? cmdPS4.L3() : guliKit.buttonLeftJoy(); }
    public Trigger joystickRight() { return isPS4 ? cmdPS4.R3() : guliKit.buttonRightJoy(); }

    public Trigger dpadUp() { return isPS4 ? cmdPS4.povUp() : guliKit.dpadUp(); }
    public Trigger dpadRight() { return isPS4 ? cmdPS4.povRight() : guliKit.dpadRight(); }
    public Trigger dpadDown() { return isPS4 ? cmdPS4.povDown() : guliKit.dpadDown(); }
    public Trigger dpadLeft() { return isPS4 ? cmdPS4.povLeft() : guliKit.dpadLeft(); }

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
    public boolean isDigitalLeft() {
        if (!isPS4) return guliKit.isDigitalLeft();
        else return true;
    }
    /** <STRONG> GULIKIT ONLY </STRONG> */
    public boolean isDigitalRight() {
        if (!isPS4) return guliKit.isDigitalRight();
        else return true;
    }
}
