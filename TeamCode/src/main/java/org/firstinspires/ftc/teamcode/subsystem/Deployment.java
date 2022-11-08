package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Deployment {

    public HardwareMap hardwareMap;
    public DcMotorEx vertical;
    public Servo claw1, claw2;

    private Height height;
    private int verticalTarget;
    private double horizontalTarget;

    public Deployment(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        vertical = hardwareMap.get(DcMotorEx.class, "vertical");
        vertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical.setTargetPosition(0);
        vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

    public void setHorizontalTarget(double target) {
        horizontalTarget = target;
        // need to calculate crank slider for distance to servo position
        // also will be multiple possible values due to two separate linkages
    }

    public void update() {
        vertical.setTargetPosition(verticalTarget);
    }

    public enum Height {
        GROUND,
        LOW,
        MIDDLE,
        HIGH
    }
}
