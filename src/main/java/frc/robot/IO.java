package frc.robot;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * The IO class manages the driver input to the robot. This includes getting axis values and binding buttons.
 */
public class IO {

    private static final int XBOX_PORT = 0;
    // Deadzone for Xbox controller sticks
    private static final double DEADZONE = 0.05;
    // This value is used to turn an analog input into a digital one so that commands can be mapped to it.
    private static final double NOMINAL_ANALOG_VALUE = 0.5;


    private XboxController xbox;

    // Custom buttons on the Xbox Controller
    private Trigger rightTriggerButton;
    private Trigger leftTriggerButton;
    private Trigger radialUp;
    private Trigger radialRight;
    private Trigger radialDown;
    private Trigger radialLeft;

    private Map<Integer, Trigger> customButtons;

    // Code for singleton
    private static IO instance;

    private IO() {}

    /**
     * Get the pre-existing instance of the IO class, or create one if there isn't one.
     * 
     * @return An instance of the IO class
     */
    public static IO getInstance() {
        if(instance == null) {
            instance = new IO();
            instance.init();
        }
        return instance;
    }

    /**
     * Initialize or reinitialize the IO class.
     * <p>
     * This does not need to be called when initializing with the getInstance() method.
     */
    public void init(){
        xbox = new XboxController(XBOX_PORT);
        initializeCustomButtons();
        customButtons = Map.of(
            11,rightTriggerButton,
            12, leftTriggerButton,
            13, radialUp,
            14, radialRight,
            15, radialDown,
            16, radialLeft
        );
    }

    /**
     * Filter a raw axis input by applying the following transformations:
     * <p>
     * <ul>
     * <li>Apply a deadzone as specified by the DEADZONE constant
     * <li>Square the raw axis value to make it feel more natural for drivers
     * </ul>
     * <hr>
     * @param input The raw axis value.
     * @return The filtered axis value.
     */
    public double filter(double input){
        double x = Math.copySign(Math.pow(input, 2), input); // Square input
        return MathUtil.applyDeadband(x, DEADZONE); // Apply deadzone and return
    }
    
    /**
     * Types of button bindings
     * 
     * @see <a href="https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/button/Button.html">See the button class for definitions of types.</a>
     */
    public enum ButtonActionType {
        WHEN_HELD,
        WHEN_PRESSED,
        WHEN_RELEASED,
        WHILE_HELD,
        TOGGLE_WHEN_PRESSED;
    }

    /**
     * Buttons on the Xbox Controller
     */
    public enum ControllerButton {
        LeftBumper(5),
        RightBumper(6),
        LeftStick(9),
        RightStick(10),
        A(1),
        B(2),
        X(3),
        Y(4),
        Back(7),
        Start(8),
        RightTriggerButton(11),
        LeftTriggerButton(12),
        RadialUp(13),
        RadialRight(14),
        RadialDown(15),
        RadialLeft(16);

        public final int VALUE;
        
        ControllerButton(int VALUE) {
            this.VALUE = VALUE;
         }
    }    

    /**
     * Bind a command to a button on the Xbox controller.
     * 
     * @param type The type of binding to perform
     * @param xboxButton The button on the Xbox controller to bind to
     * @param command The command to bind
     */
    public void bind(ButtonActionType type, ControllerButton xboxButton, CommandBase command) {
        Trigger button;
        if (xboxButton.VALUE >= 11) {
            button = customButtons.get(xboxButton.VALUE);
        } else {
            button = new JoystickButton(xbox, xboxButton.VALUE);
        }
        
        switch(type) {
            case TOGGLE_WHEN_PRESSED:
                button.toggleOnTrue(command);
                break;
            case WHEN_HELD:
                button.whileTrue(command);
                break;
            case WHEN_PRESSED:
                button.onTrue(command);
                break;
            case WHEN_RELEASED:
                button.onFalse(command);
                break;
            case WHILE_HELD:
                button.whileTrue(new RepeatCommand(command));
                break;
        }
    }

    /**
     * Initialize the custom buttons
     */
    public void initializeCustomButtons(){
        rightTriggerButton = new Trigger( () -> {
            return xbox.getRightTriggerAxis() > NOMINAL_ANALOG_VALUE; 
        } );

        leftTriggerButton = new Trigger( () -> {
            return xbox.getLeftTriggerAxis() > NOMINAL_ANALOG_VALUE;
        } );

        radialUp = new Trigger( () -> {
            return xbox.getPOV() == 0;
        } );

        radialRight = new Trigger( () -> {
            return xbox.getPOV() == 90;
        } );

        radialDown = new Trigger( () -> {
            return xbox.getPOV() == 180;
        } );

        radialLeft = new Trigger( () -> {
            return xbox.getPOV() == 270;
        } );
    }
    
    /**
     * @return Get the horizontal axis of the left stick on the Xbox controller
     */
    public double getLeftX(){
        return filter(xbox.getLeftX());
    }

    /**
     * @return Get the horizontal axis of the right stick on the Xbox controller
     */
    public double getRightX(){
        return filter(xbox.getRightX());
    }
    
    /**
     * @return Get the vertical axis of the left stick on the Xbox controller
     */
    public double getLeftY(){
        return filter(-xbox.getLeftY());
    }

    /**
     * @return Get the vertical axis of the right stick on the Xbox controller
     */
    public double getRightY(){
        return filter(-xbox.getRightY());
    }
}


