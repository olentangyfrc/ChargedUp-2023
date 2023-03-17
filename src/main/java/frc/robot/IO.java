package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * The IO class manages the driver input to the robot. This includes getting axis values and binding buttons.
 */
public class IO {

    private static final int XBOX_PORT = 0;
    private static final int AUX_XBOX_PORT = 1;

    public static final int LEFT_BUTTON_BOX_PORT = 3;
    public static final int RIGHT_BUTTON_BOX_PORT = 4;
    // Deadzone for Xbox controller sticks
    private static final double DEADZONE = 0.02;
    // This value is used to turn an analog input into a digital one so that commands can be mapped to it.
    private static final double NOMINAL_ANALOG_VALUE = 0.5;

    private GenericHID leftButtonBox;
    private GenericHID rightButtonBox;

    private CommandXboxController xbox;

    // Custom buttons on the Xbox Controller
    private Trigger rightTriggerButton;
    private Trigger leftTriggerButton;
    private Trigger radialUp;
    private Trigger radialRight;
    private Trigger radialDown;
    private Trigger radialLeft;

    private Trigger leftYPos;
    private Trigger leftYNeg;
    private Trigger leftXPos;
    private Trigger leftXNeg;

    private Trigger rightYPos;
    private Trigger rightYNeg;
    private Trigger rightXPos;
    private Trigger rightXNeg;



    private Map<Integer, Trigger> customButtons;

    // Code for singleton
    private static IO instance;
    private static IO auxInstance;

    private IO() {}

    /**
     * Get the pre-existing instance of the IO class, or create one if there isn't one.
     * 
     * @return An instance of the IO class
     */
    public static IO getInstance() {
        if(instance == null) {
            instance = new IO();
            instance.init(false);
        }
        return instance;
    }

    public static IO getAuxInstance() {
        if(auxInstance == null) {
            auxInstance = new IO();
            auxInstance.init(true);
        }

        return auxInstance;
    }

    /**
     * Initialize or reinitialize the IO class.
     * <p>
     * This does not need to be called when initializing with the getInstance() method.
     */
    public void init(boolean isAux){
        xbox = new CommandXboxController(isAux? AUX_XBOX_PORT : XBOX_PORT);

        initializeCustomButtons();
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

    public boolean getRightStick() {
        return xbox.rightStick().getAsBoolean();
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
        A(1),
        B(2),
        X(3),
        Y(4),
        LeftBumper(5),
        RightBumper(6),
        Back(7),
        Start(8),
        LeftStick(9),
        RightStick(10),
        RightTriggerButton(11),
        LeftTriggerButton(12),
        RadialUp(13),
        RadialRight(14),
        RadialDown(15),
        RadialLeft(16),
        leftYPos(17),
        leftYNeg(18),
        leftXPos(19),
        leftXNeg(20),
        rightYPos(21),
        rightYNeg(22),
        rightXPos(23),
        rightXNeg(24);

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
            button = new JoystickButton(xbox.getHID(), xboxButton.VALUE);
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
     * Bind a button to the button box.
     * <p>
     * Do not look at this code as an example! It is repetitive, but competition is in a week.
     * 
     * @param command The command to assign
     * @param buttonBoxButton The button to assign to.
     * @param type The type of action
     */
    public void bindButtonBox(CommandBase command, StickButton buttonBoxButton, ButtonActionType type) {
        GenericHID buttonBoxSide;
        int buttonNum;

        if(buttonBoxButton.ordinal() <= 10) {
            buttonBoxSide = leftButtonBox;
            buttonNum = buttonBoxButton.ordinal() + 1;
        } else {
            buttonBoxSide = rightButtonBox;
            buttonNum = buttonBoxButton.ordinal() - 10;
        }

        JoystickButton button = new JoystickButton(buttonBoxSide, buttonNum);

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
            return xbox.pov(0).getAsBoolean();
        } );

        radialRight = new Trigger( () -> {
            return xbox.pov(90).getAsBoolean();
        } );

        radialDown = new Trigger( () -> {
            return xbox.pov(180).getAsBoolean();
        } );
  
        radialLeft = new Trigger( () -> {
            return xbox.pov(270).getAsBoolean();
        } );

        leftYPos = new Trigger(() -> {
            return xbox.getLeftY() < -NOMINAL_ANALOG_VALUE;
        });
        leftYNeg = new Trigger(() -> {
            return xbox.getLeftY() > NOMINAL_ANALOG_VALUE;
        });
        leftXPos = new Trigger(() -> {
            return xbox.getLeftX() > NOMINAL_ANALOG_VALUE;
        });
        leftXNeg = new Trigger(() -> {
            return xbox.getLeftX() < -NOMINAL_ANALOG_VALUE;
        });
        rightYPos = new Trigger(() -> {
            return xbox.getRightY() < -NOMINAL_ANALOG_VALUE;
        });
        rightYNeg = new Trigger(() -> {
            return xbox.getRightY() > NOMINAL_ANALOG_VALUE;
        });
        rightXPos = new Trigger(() -> {
            return xbox.getRightX() > NOMINAL_ANALOG_VALUE;
        });
        rightXNeg = new Trigger(() -> {
            return xbox.getRightTriggerAxis() < -NOMINAL_ANALOG_VALUE;
        });

        customButtons = new HashMap<Integer, Trigger>();

        customButtons.put(11,rightTriggerButton);
        customButtons.put(12, leftTriggerButton);
        customButtons.put(13, radialUp);
        customButtons.put(14, radialRight);
        customButtons.put(15, radialDown);
        customButtons.put(16, radialLeft);
        customButtons.put(17, leftYPos);
        customButtons.put(18, leftYNeg);
        customButtons.put(19, leftXPos);
        customButtons.put(20, leftXNeg);
        customButtons.put(21, rightYPos);
        customButtons.put(22, rightYNeg);
        customButtons.put(23, rightXPos);
        customButtons.put(24, rightXNeg);
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

    /**
     * The joystick buttons
     */
    public enum StickButton {
        LEFT_1,
        LEFT_2,
        LEFT_3,
        LEFT_4,
        LEFT_5,
        LEFT_6,
        LEFT_7,
        LEFT_8,
        LEFT_9,
        LEFT_10,
        LEFT_11,
        RIGHT_1,
        RIGHT_2,
        RIGHT_3,
        RIGHT_4,
        RIGHT_5,
        RIGHT_6,
        RIGHT_7,
        RIGHT_8,
        RIGHT_9,
        RIGHT_10,
        RIGHT_11,
    }
}


