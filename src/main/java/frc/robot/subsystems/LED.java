package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.Colors;
import frc.robot.Constants.LEDConstants.LEDStrips;

public class LED extends SubsystemBase {
    private final CANdle candle = new CANdle(Constants.LEDConstants.CANdleID, Constants.ModuleConstants.kCANivoreName);
    private double brightness = 1.0; // multiplier on brightness
    private boolean paused = false;

    private Color[] stripColors = new Color[LEDConstants.CANdleLength];

    private State state = State.DISABLED;
    public enum State {
        DISABLED, // when the robot is disabled (nerdherd blue to white fade)
        TELEOP, // alliance color + white scrol (RED WHITE BLUE 🦅🦅)
        AUTO, // fire 🔥🔥🔥🔥
        DISCONNECTED,
        HAS_NOTE, // note is in indexer (blue)
        LOCKED_ON, // sees april tag (yellow)
        SHOOTING // fill green
    }

    public LED() {
        candle.clearAnimation(0);
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(configAll, 100); 
        
        setStrip(toPattern(Colors.BLACK), LEDStrips.ALL);
    }

    private void setLED(int r, int g, int b, int index) {
        candle.setLEDs((int)(r * brightness), (int)(g * brightness), (int)(b * brightness), 0, index, 0);
    }
    private void setLED(Color color, int index) {
        candle.setLEDs((int)(color.red * 255 * brightness), (int)(color.green * 255 * brightness), (int)(color.blue * 255 * brightness), 0, index, 0);
    }

    private void setLEDs(int r, int g, int b, int index, int count) {
        candle.setLEDs((int)(r * brightness), (int)(g * brightness), (int)(b * brightness), 0, index, count);
    }
    private void setLEDs(int r, int g, int b, LEDStrips section) {
        setLEDs((int)(r * brightness), (int)(g * brightness), (int)(b * brightness), section.index, section.count);
    }
    private void setLEDs(Color color, int index, int count) {
        candle.setLEDs((int)(color.red * 255 * brightness), (int)(color.green * 255 * brightness), (int)(color.blue * 255 * brightness), 0, index, count);
    }
    private void setLEDs(Color color, LEDStrips section) {
        setLEDs((int)(color.red * 255 * brightness), (int)(color.green * 255 * brightness), (int)(color.blue * 255 * brightness), section.index, section.count);
    }
    
    private void setLEDs(int r, int g, int b) {
        setLEDs((int)(r * brightness), (int)(g * brightness), (int)(b * brightness));
    }
    private void setLEDs(Color color) {
        setLEDs((int)(color.red * 255 * brightness), (int)(color.green * 255 * brightness), (int)(color.blue * 255 * brightness));
    }

    public void setState(State _state){
        state = _state;
    }
    public Command setStateCommand(State _state){
        return Commands.runOnce(()->setState(_state));
    }
    public Command setStateDisabledCommand(){
        return Commands.runOnce(()->setState(State.DISABLED));
    }
    public Command setStateTeleopCommand(){
        return Commands.runOnce(()->setState(State.TELEOP));
    }
    public Command setStateAutoCommand(){
        return Commands.runOnce(()->setState(State.AUTO));
    }
    public Command setStateDisconnectedCommand(){
        return Commands.runOnce(()->setState(State.DISCONNECTED));
    }
    public Command setStateHasNoteCommand(){
        return Commands.runOnce(()->setState(State.HAS_NOTE));
    }
    public Command setStateShooterReadyCommand(){
        return Commands.runOnce(()->setState(State.SHOOTING));
    }
    
    /**
     * pauses the LEDs
     */
    public void pause() {
        paused = true;
    }
    /**
     * pauses the LEDs
     */
    public Command pauseCommand() {
        return Commands.runOnce(()->pause());
    }
    /**
     * unpauses the LEDs
     */
    public void start() {
        paused = false;
    }
    /**
     * unpauses the LEDs
     */
    public Command startCommand() {
        return Commands.runOnce(()->start());
    }
    /**
     * pauses and clears the LEDs (effectively turns them off)
     */
    public void disable() {
        paused = false;
        setLEDs(Colors.BLACK);
    }
    /**
     * pauses and clears the LEDs (effectively turns them off)
     */
    public Command disableCommand() {
        return Commands.runOnce(()->disable());
    }

    /**
     * sets section to pattern
     * @param colors pattern to set
     * @param index index of section
     * @param count count of section
     * @param rotation how much to rotate the colors in the strip
     */
    private void setStrip(Color[] colors, int index, int count, int rotation) {
        for (int i = 0; i < count; i++) {
            stripColors[index + i] = colors[Math.abs(Math.floorMod(i + rotation, colors.length))];
        }
    }
    /**
     * sets section to pattern
     * @param colors pattern to set
     * @param index index of section
     * @param count count of section
     */
    private void setStrip(Color[] colors, int index, int count) {
        setStrip(colors, index, count, 0);
    }
    /**
     * setStrip based on section
     * @param colors pattern to set
     * @param section section to set
     * @param rotation rotation of pattern
     */
    private void setStrip(Color[] colors, LEDStrips section, int rotation){
        setStrip(colors, section.index, section.count, rotation);
    }
    /**
     * setStrip based on section
     * @param colors
     * @param section
     */
    private void setStrip(Color[] colors, LEDStrips section){
        setStrip(colors, section.index, section.count);
    }

    /**
     * used for easier array creation due to annoying syntax
     * @param color
     * @return an array with the single color
     */
    private Color[] toPattern(Color color) { return new Color[]{color}; }
    /**
     * creates new color object (feels better than writing new every time)
     * @return unique color object
     */
    private static Color Color(double r, double g, double b) { return new Color(r, g, b); }

    /* Wrappers so we can access the CANdle from the subsystem */
    public double getVbat() { return candle.getBusVoltage(); }
    public double get5V() { return candle.get5VRailVoltage(); }
    public double getCurrent() { return candle.getCurrent(); }
    public double getTemperature() { return candle.getTemperature(); }
    public void configBrightness(double percent) { candle.configBrightnessScalar(percent, 0); }
    public void configLos(boolean disableWhenLos) { candle.configLOSBehavior(disableWhenLos, 0); }
    public void configLedType(LEDStripType type) { candle.configLEDType(type, 0); }
    public void configStatusLedBehavior(boolean offWhenActive) { candle.configStatusLedState(offWhenActive, 0); }
    
    /**
     * linear interpolate between two colors
     * @param a first color
     * @param b second color
     * @param t [0,1] lerp coefficient
     * @return
     */
    private Color lerpColor(Color a, Color b, double t) {
        return Color(a.red * (1.0 - t) + b.red * t,
                    a.green * (1.0 - t) + b.green * t,
                    a.blue * (1.0 - t) + b.blue * t);
    }

    /**
     * applies strip colors
     */
    private void updateCANdle() {
        for (int i = 0; i < stripColors.length; i++) {
            setLED(stripColors[i], i);
        }
    }

    private void updateState() {
        if (RobotState.isDisabled()) setState(State.DISABLED);
        // else if (something) setState(State.SHOOTING)
        // else if (other) setState(State.LOCKED_ON)
        // else if (idk) setState(State.HAS_NOTE);
        else if (RobotState.isTeleop()) setState(State.TELEOP);
        else if (RobotState.isAutonomous()) setState(State.AUTO);
    }

    private int delay = 0;
    private int time = 0; // time value
    // Runs 5 times a second
    @Override
    public void periodic() {
        if (!paused) delay++;
        if (delay >= 10 && !paused) { delay = 0;
            time++;
            updateState();
            setStrip(toPattern(Colors.BLACK), LEDStrips.ALL); // clear strip
            // draws go here
            switch (state) {
                case TELEOP:
                    
                    break;
                default:
                    double t = (Math.sin(time / 10.0) + 1.0) / 2.0;
                    setStrip(toPattern(lerpColor(Colors.WHITE, Colors.NERDHERD_BLUE, t)), LEDStrips.ALL);
                    break;
            }
            
            updateCANdle(); // push draws
        }
    }

    
}