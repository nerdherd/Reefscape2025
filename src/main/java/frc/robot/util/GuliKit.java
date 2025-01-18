package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class GuliKit {
    private final Joystick ma_joystick;
    private final JoystickButton trigger, thumbButton, button3,
                                 button4, button5, button6,
                                 button7, button8, button9,
                                 button10, button11, button12;

    public GuliKit(int port) {
        ma_joystick = new Joystick(port);
        trigger = new JoystickButton(ma_joystick, 1);
        thumbButton = new JoystickButton(ma_joystick, 2);
        button3 = new JoystickButton(ma_joystick, 3);
        button4 = new JoystickButton(ma_joystick, 4);
        button5 = new JoystickButton(ma_joystick, 5);
        button6 = new JoystickButton(ma_joystick, 6);
        button7 = new JoystickButton(ma_joystick, 7);
        button8 = new JoystickButton(ma_joystick, 8);
        button9 = new JoystickButton(ma_joystick, 9);
        button10 = new JoystickButton(ma_joystick, 10);
        button11 = new JoystickButton(ma_joystick, 11);
        button12 = new JoystickButton(ma_joystick, 12);
    }

    public JoystickButton getTrigger() {return trigger;}
    public JoystickButton getThumbButtn() {return thumbButton;}
    public JoystickButton getButton3() {return button3;} 
    public JoystickButton getButton4() {return button4;}
    public JoystickButton getButton5() {return button5;}
    public JoystickButton getButton6() {return button6;}
    public JoystickButton getButton7() {return button7;}
    public JoystickButton getButton8() {return button8;}
    public JoystickButton getButton9() {return button9;}
    public JoystickButton getButton10() {return button10;}
    public JoystickButton getButton11() {return button11;}
    public JoystickButton getButton12() {return button12;}

    public double getXAxis() { return ma_joystick.getRawAxis(0); }
    public double getYAxis() { return ma_joystick.getRawAxis(1); }
    public double getZAxis() { return ma_joystick.getRawAxis(2); }
    public double getSliderAxis() { return ma_joystick.getRawAxis(3); }
}
