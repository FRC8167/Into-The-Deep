package org.firstinspires.ftc.teamcode.Cogintilities;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadWrapper {

    public DigitalInput a = new DigitalInput(false, false);
    public DigitalInput b = new DigitalInput(false, false);
    public DigitalInput x = new DigitalInput(false, false);
    public DigitalInput y = new DigitalInput(false, false);

    public DigitalInput start = new DigitalInput(false, false);
    public DigitalInput back  = new DigitalInput(false, false);
    public DigitalInput guide = new DigitalInput(false, false);

    public DigitalInput dpadUp    = new DigitalInput(false, false);
    public DigitalInput dpadDown  = new DigitalInput(false, false);
    public DigitalInput dpadLeft  = new DigitalInput(false, false);
    public DigitalInput dpadRight = new DigitalInput(false, false);

    public DigitalInput leftBumper  = new DigitalInput(false, false);
    public DigitalInput rightBumper = new DigitalInput(false, false);

    public double leftStick_X, leftStick_Y, rightStick_X, rightStick_Y, leftTrigger, rightTrigger;
    public double leftJoyMagnitude, rightJoyMagnitude;

    private Gamepad gamepad;

    /**
     *
     * @param gamepad
     */
    public GamepadWrapper(Gamepad gamepad) { this.gamepad = gamepad; }

    /**
     * This method should be called once per opMode loop.
     */
    public void update() {

        leftStick_X  = gamepad.left_stick_x;
        leftStick_Y  = gamepad.left_stick_y;
        rightStick_X = gamepad.right_stick_x;
        rightStick_Y = gamepad.right_stick_y;
        leftTrigger  = gamepad.left_trigger;
        rightTrigger = gamepad.right_trigger;

        a.update(gamepad.a);
        b.update(gamepad.b);
        x.update(gamepad.x);
        y.update(gamepad.y);
        dpadUp.update(gamepad.dpad_up);
        dpadDown.update(gamepad.dpad_down);
        dpadLeft.update(gamepad.dpad_left);
        dpadRight.update(gamepad.dpad_right);
        leftBumper.update(gamepad.left_bumper);
        rightBumper.update(gamepad.right_bumper);
        back.update(gamepad.back);
        start.update(gamepad.start);
        guide.update(gamepad.guide);

        leftJoyMagnitude  = Math.sqrt(Math.pow(gamepad.left_stick_x, 2) + Math.pow(gamepad.left_stick_y, 2));
        rightJoyMagnitude = Math.sqrt(Math.pow(gamepad.right_stick_x, 2) + Math.pow(gamepad.right_stick_y, 2));
    }

}
