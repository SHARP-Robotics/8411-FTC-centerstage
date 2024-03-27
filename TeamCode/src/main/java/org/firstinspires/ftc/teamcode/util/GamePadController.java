package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamePadController {
    private final double triggerThreshold = 0.8;
    private final double joystickThreshold = 0.9;

    private boolean active = true;

    private Gamepad gamepad;

    private int dpad_up, dpad_down, dpad_left, dpad_right;
    private int x, y, a, b;
    private int left_bumper, right_bumper;
    private int start, back, guide;
    private int left_trigger_it, right_trigger_it;
    private int left_stick_button, right_stick_button;
    private int left_stick_px_it, left_stick_nx_it, left_stick_py_it, left_stick_ny_it;
    private int right_stick_px_it, right_stick_nx_it, right_stick_py_it, right_stick_ny_it;

    public double left_stick_x, right_stick_x, left_stick_y, right_stick_y;
    public double left_trigger, right_trigger;

    private boolean bReleased = false;

    public GamePadController(Gamepad originalGamepad) {
        gamepad = originalGamepad;
    }

    public void setGamepad(Gamepad newGamepad) {
        gamepad = newGamepad;
    }

    public void enable() {
        active = true;
        update();
    }
    public void disable() {
        active = false;
        update();
        update();
    }
    public boolean isActive() {
        return active;
    }

    public void update() {
        // Simple booleans
        if (gamepad.x && active) { ++x; } else { x = 0; }
        if (gamepad.y && active) { ++y; } else { y = 0; }
        if (gamepad.a && active) { ++a; } else { a = 0; }
        if (gamepad.b && active) { ++b; } else { bReleased = b > 0; b = 0; }
        if (gamepad.dpad_up && active) { ++dpad_up; } else { dpad_up = 0; }
        if (gamepad.dpad_down && active) { ++dpad_down; } else { dpad_down = 0; }
        if (gamepad.dpad_left && active) { ++dpad_left; } else { dpad_left = 0; }
        if (gamepad.dpad_right && active) { ++dpad_right; } else { dpad_right = 0; }
        if (gamepad.left_bumper && active) { ++left_bumper; } else { left_bumper = 0; }
        if (gamepad.right_bumper && active) { ++right_bumper; } else { right_bumper = 0; }
        if (gamepad.start && active) { ++start; } else { start = 0; }
        if (gamepad.back && active) { ++back; } else { back = 0; }
        if (gamepad.guide && active) { ++guide; } else { guide = 0; }
        if (gamepad.left_stick_button && active) { ++left_stick_button; } else { left_stick_button = 0; }
        if (gamepad.right_stick_button && active) { ++right_stick_button; } else { right_stick_button = 0; }

        // Threshold Triggers
        if (gamepad.left_trigger > triggerThreshold && active) { ++left_trigger_it; } else { left_trigger_it = 0; }
        if (gamepad.right_trigger > triggerThreshold && active) { ++right_trigger_it; } else { right_trigger_it = 0; }

        // Threshold Left stick
        if (gamepad.left_stick_x < -joystickThreshold && active) { ++left_stick_nx_it; } else { left_stick_nx_it = 0; }
        if (gamepad.left_stick_x > joystickThreshold && active) { ++left_stick_px_it; } else { left_stick_px_it = 0; }
        if (gamepad.left_stick_y < -joystickThreshold && active) { ++left_stick_ny_it; } else { left_stick_ny_it = 0; }
        if (gamepad.left_stick_y > joystickThreshold && active) { ++left_stick_py_it; } else { left_stick_py_it = 0; }

        // Threshold Right stick
        if (gamepad.right_stick_x < -joystickThreshold && active) { ++right_stick_nx_it; } else { right_stick_nx_it = 0; }
        if (gamepad.right_stick_x > joystickThreshold && active) { ++right_stick_px_it; } else { right_stick_px_it = 0; }
        if (gamepad.right_stick_y < -joystickThreshold && active) { ++right_stick_ny_it; } else { right_stick_ny_it = 0; }
        if (gamepad.right_stick_y > joystickThreshold && active) { ++right_stick_py_it; } else { right_stick_py_it = 0; }

        // Variables
        if (active) { left_stick_x = gamepad.left_stick_x; } else { left_stick_x = 0; }
        if (active) { left_stick_y = gamepad.left_stick_y; } else { left_stick_y = 0; }
        if (active) { right_stick_x = gamepad.right_stick_x; } else { right_stick_x = 0; }
        if (active) { right_stick_y = gamepad.right_stick_y; } else { right_stick_y = 0; }
        if (active) { left_trigger = gamepad.left_trigger; } else { left_trigger = 0; }
        if (active) { right_trigger = gamepad.right_trigger; } else { right_trigger = 0; }
    }

    public boolean dpadUp() { return 0 < dpad_up; }
    public boolean dpadDown() { return 0 < dpad_down; }
    public boolean dpadLeft() { return 0 < dpad_left; }
    public boolean dpadRight() { return 0 < dpad_right; }
    public boolean x() { return 0 < x; }
    public boolean y() { return 0 < y; }
    public boolean a() { return 0 < a && !start(); }
    public boolean b() { return 0 < b && !start(); }
    public boolean leftBumper() { return 0 < left_bumper; }
    public boolean rightBumper() { return 0 < right_bumper; }
    public boolean leftTrigger() { return 0 < left_trigger_it;}
    public boolean rightTrigger() { return 0 < right_trigger_it;}
    public boolean leftStickButton() { return 0 < left_stick_button; }
    public boolean rightStickButton() { return 0 < right_stick_button; }
    public boolean leftStickRight() { return 0 < left_stick_px_it; }
    public boolean leftStickUp() { return 0 < left_stick_ny_it; }
    public boolean leftStickLeft() { return 0 < left_stick_nx_it; }
    public boolean leftStickDown() { return 0 < left_stick_py_it; }
    public boolean rightStickRight() { return 0 < right_stick_px_it; }
    public boolean rightStickUp() { return 0 < right_stick_ny_it; }
    public boolean rightStickLeft() { return 0 < right_stick_nx_it; }
    public boolean rightStickDown() { return 0 < right_stick_py_it; }

    public boolean xLong() { return 10 < x; }
    public boolean yLong() { return 10 < y; }
    public boolean aLong() { return 10 < a; }
    public boolean bLong() { return 10 < b; }
    public boolean rightBumperLong() { return 10 < right_bumper; }
    public boolean leftBumperLong() { return 10 < left_bumper; }

    public boolean dpadDownLong() { return 5 < dpad_down; }
    public boolean dpadDownLongOnce() { return 10 == dpad_down; }

    public boolean dpadUpLong() { return 5 < dpad_up; }
    public boolean dpadUpLongOnce() { return 10 == dpad_up; }

    public boolean dpadLeftLong() { return 10 < dpad_left; }
    public boolean dpadLeftLongOnce() { return 10 == dpad_left; }

    public boolean dpadRightLong() { return 10 < dpad_right; }
    public boolean dpadRightLongOnce() { return 10 == dpad_right; }

    public boolean xLongOnce() { return 10 == x; }
    public boolean yLongOnce() { return 10 == y; }
    public boolean aLongOnce() { return 10 == a; }
    public boolean bLongOnce() { return 10 == b; }
    public boolean rightBumperLongOnce() { return 10 == right_bumper; }
    public boolean leftBumperLongOnce() { return 10 == left_bumper; }

    public boolean dpadUpOnce() { return 1 == dpad_up; }
    public boolean dpadDownOnce() { return 1 == dpad_down; }
    public boolean dpadLeftOnce() { return 1 == dpad_left; }
    public boolean dpadRightOnce() { return 1 == dpad_right; }
    public boolean xOnce() { return 1 == x; }
    public boolean yOnce() { return 1 == y; }
    public boolean aOnce() { return 1 == a && !start(); }
    public boolean bOnce() { return 1 == b && !start(); }
    public boolean leftBumperOnce() { return 1 == left_bumper; }
    public boolean rightBumperOnce() { return 1 == right_bumper; }
    public boolean leftTriggerOnce() { return 1 == left_trigger_it; }
    public boolean rightTriggerOnce() { return 1 == right_trigger_it; }
    public boolean leftStickButtonOnce() { return 1 == left_stick_button; }
    public boolean rightStickButtonOnce() { return 1 == right_stick_button; }
    public boolean leftStickRightOnce() { return 1 == left_stick_px_it; }
    public boolean leftStickUpOnce() { return 1 == left_stick_ny_it; }
    public boolean leftStickLeftOnce() { return 1 == left_stick_nx_it; }
    public boolean leftStickDownOnce() { return 1 == left_stick_py_it; }
    public boolean rightStickRightOnce() { return 1 == right_stick_px_it; }
    public boolean rightStickUpOnce() { return 1 == right_stick_ny_it; }
    public boolean rightStickLeftOnce() { return 1 == right_stick_nx_it; }
    public boolean rightStickDownOnce() { return 1 == right_stick_py_it; }

    public boolean leftStickButtonLong() { return 10 < left_stick_button; }
    public boolean rightStickButtonLong() { return 10 < right_stick_button; }
    public boolean leftStickRightLong() { return 10 < left_stick_px_it; }
    public boolean leftStickUpLong() { return 10 < left_stick_ny_it; }
    public boolean leftStickLeftLong() { return 10 < left_stick_nx_it; }
    public boolean leftStickDownLong() { return 10 < left_stick_py_it; }
    public boolean rightStickRightLong() { return 10 < right_stick_px_it; }
    public boolean rightStickUpLong() { return 10 < right_stick_ny_it; }
    public boolean rightStickLeftLong() { return 10 < right_stick_nx_it; }
    public boolean rightStickDownLong() { return 10 < right_stick_py_it; }

    public boolean start() {return 0 < start;}
    public boolean startOnce() {return 1 == start;}
    public boolean back() {return 0 < back;}
    public boolean backOnce() {return 1 == back;}
    public boolean guide() {return 0 < guide;}
    public boolean guideOnce() {return 1 == guide;}

    public boolean guideLong() {return 5< guide;}

    public boolean bReleased() {return bReleased;}

    /**
     * Reset all iteration counts to zero.
     */
    public void reset() {
        disable();
        enable();
    }
    /* Rumble Support */

    /**
     * Rumble the gamepad's first rumble motor at maximum power for a certain duration.
     * Calling this will displace any currently running rumble effect.
     * @param durationMs milliseconds to rumble for, or -1 for continuous
     */
    public void rumble(int durationMs) {
        gamepad.rumble(durationMs);
    }

    /**
     * Rumble the gamepad at a fixed rumble power for a certain duration
     * Calling this will displace any currently running rumble effect
     * @param rumble1 rumble power for rumble motor 1 (0.0 - 1.0)
     * @param rumble2 rumble power for rumble motor 2 (0.0 - 1.0)
     * @param durationMs milliseconds to rumble for, or -1 for continuous
     */
    public void rumble(double rumble1, double rumble2, int durationMs) {
        gamepad.rumble(rumble1, rumble2, durationMs);
    }

    /**
     * Rumble the gamepad for a certain number of "blips" using predetermined blip timing
     * This will displace any currently running rumble effect.
     * @param count the number of rumble blips to perform
     */
    public void rumbleBlips(int count) {
        gamepad.rumbleBlips(count);
    }

    /**
     * Run a rumble effect built using {@link Gamepad.RumbleEffect.Builder}
     * The rumble effect will be run asynchronously; your OpMode will
     * not halt execution while the effect is running.
     *
     * Calling this will displace any currently running rumble effect
     */
    public void runRumbleEffect(Gamepad.RumbleEffect effect) {
        gamepad.runRumbleEffect(effect);
    }

    /**
     * Cancel the currently running rumble effect, if any
     */
    public void stopRumble() {
        gamepad.stopRumble();
    }

    /**
     * Returns an educated guess about whether there is a rumble action ongoing on this gamepad
     * @return an educated guess about whether there is a rumble action ongoing on this gamepad
     */
    public boolean isRumbling() {
        return gamepad.isRumbling();
    }

    public Gamepad.RumbleEffect.Builder effectBuilder() {
        return new Gamepad.RumbleEffect.Builder();
    }

    public void runLedEffect(Gamepad.LedEffect ledEffect) {
        gamepad.runLedEffect(ledEffect);
    }

    public void setLedColor(double r, double g, double b, int durationMs) {
        gamepad.setLedColor(r,g,b,durationMs);
    }
}