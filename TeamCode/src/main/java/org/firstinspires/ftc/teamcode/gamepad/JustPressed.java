package org.firstinspires.ftc.teamcode.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Collection;
import java.util.HashSet;

public class JustPressed {

    public Gamepad gamepad;
    public GamepadListenerEx listener;
    public Collection<GamepadListener.Button> justPressed;

    public JustPressed(Gamepad gamepad) {
        this.gamepad = gamepad;
        this.listener = new GamepadListenerEx(gamepad) {
            @Override
            public void onButtonPress(Button button) {
                super.onButtonPress(button);
                justPressed.add(button);
            }
        };
    }

    public void update() {
        justPressed = new HashSet<>();
        listener.update();
    }

    public boolean a() {
        return justPressed.contains(GamepadListener.Button.a);
    }
    public boolean b() {
        return justPressed.contains(GamepadListener.Button.b);
    }
    public boolean x() {
        return justPressed.contains(GamepadListener.Button.x);
    }
    public boolean y() {
        return justPressed.contains(GamepadListener.Button.y);
    }
    public boolean dpad_up() {
        return justPressed.contains(GamepadListener.Button.dpad_up);
    }
    public boolean dpad_down() {
        return justPressed.contains(GamepadListener.Button.dpad_down);
    }
    public boolean dpad_left() {
        return justPressed.contains(GamepadListener.Button.dpad_left);
    }
    public boolean dpad_right() {
        return justPressed.contains(GamepadListener.Button.dpad_right);
    }
    public boolean left_bumper() {
        return justPressed.contains(GamepadListener.Button.left_bumper);
    }
    public boolean right_bumper() {
        return justPressed.contains(GamepadListener.Button.right_bumper);
    }
    public boolean left_stick_button() {
        return justPressed.contains(GamepadListener.Button.left_stick_button);
    }
    public boolean right_stick_button() {
        return justPressed.contains(GamepadListener.Button.right_stick_button);
    }
    public boolean start() {
        return justPressed.contains(GamepadListener.Button.start);
    }
    public boolean guide() {
        return justPressed.contains(GamepadListener.Button.guide);
    }
}