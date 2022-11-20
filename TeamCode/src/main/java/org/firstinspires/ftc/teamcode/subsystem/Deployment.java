package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Deployment {

    public HardwareMap hardwareMap;
    public DcMotorEx slides;
    public Servo claw1, claw2;

    private Height height;
    private int verticalTarget;
    private Claw claw;

    public Deployment(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        slides = hardwareMap.get(DcMotorEx.class, "slides");
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setTargetPosition(0);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        claw1 = hardwareMap.get(Servo.class, "claw1");
        claw2 = hardwareMap.get(Servo.class, "claw2");
    }

    public Height getHeight() {
        return height;
    }

    public void setHeight(Height height) {
        this.height = height;
        // verticalTarget = ...
        // linear and straight forward. just need to do pid
    }

    public void grabCone() {
        claw = Claw.CLOSE;
    }

    public void releaseCone() {
        claw = Claw.OPEN;
    }

    public void update() {
        slides.setTargetPosition(verticalTarget);

        switch (claw) {
            case OPEN:
                claw1.setPosition(0);
                claw2.setPosition(0);
                break;
            case CLOSE:
                claw1.setPosition(1);
                claw2.setPosition(1);
        }
    }

    public enum Height {
        GROUND,
        LOW,
        MIDDLE,
        HIGH
    }

    public enum Claw {
        OPEN,
        CLOSE
    }
}
