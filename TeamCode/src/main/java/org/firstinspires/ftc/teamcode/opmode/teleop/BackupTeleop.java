package org.firstinspires.ftc.teamcode.opmode.teleop;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystem.Constants;

//@Disabled
@TeleOp
public class BackupTeleop extends LinearOpMode{
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotorEx slides;
    private Servo claw1;
    private Servo claw2;


    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        slides = hardwareMap.get(DcMotorEx.class, "slides");
        claw1 = hardwareMap.get(Servo.class, "claw1");
        claw2 = hardwareMap.get(Servo.class, "claw2");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setTargetPosition(0);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // slides.setTargetPositionTolerance(50);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        int slidesTarget = Constants.SLIDES_DOWN;
        double slidesPower = 0;

        boolean aPressed = false;
        boolean clawClosed = false;

        double forward = 0;
        double turn = 0;
        double strafe = 0;
        double heading = 0;

        waitForStart();

        while(opModeIsActive())
        {
            heading = 0; //-imu.getAngularOrientation().firstAngle;
            forward = -cube(gamepad1.left_stick_y) * .75;
            turn = cube(gamepad1.right_stick_x) * .80;
            strafe = cube(gamepad1.left_stick_x) * .75;
            if (gamepad1.right_bumper || slidesTarget > 500) {
                forward *= .35;
                turn *= .35;
                strafe *= .35;
            }

            double rotX = strafe * Math.cos(heading) - forward * Math.sin(heading);
            double rotY = strafe * Math.sin(heading) + forward * Math.cos(heading);

            double denominator = Math.max(Math.abs(forward) + Math.abs(turn) + Math.abs(strafe), 1);

            leftFront.setPower((rotY + rotX + turn) / denominator);
            rightFront.setPower((rotY - rotX - turn) / denominator);
            leftBack.setPower((rotY - rotX + turn) / denominator);
            rightBack.setPower((rotY + rotX - turn) / denominator);

            if (gamepad1.dpad_down) {
                slidesTarget = Constants.SLIDES_DOWN;
                slidesPower = .65;
            } else if (gamepad1.dpad_left) {
                slidesTarget = Constants.SLIDES_LOW;
                slidesPower = .6;
            } else if (gamepad1.dpad_right) {
                slidesTarget = Constants.SLIDES_MID;
                slidesPower = .6;
            } else if (gamepad1.dpad_up) {
                slidesTarget = Constants.SLIDES_HIGH;
                slidesPower = .7;
            } else if (gamepad1.left_bumper) {
                slidesTarget = Constants.SLIDES_GROUND;
                slidesPower = .5;
            }

            slidesTarget += gamepad1.right_trigger * 1.5 - gamepad1.left_trigger * 1.5;

            if (Math.abs(slides.getCurrentPosition() - slidesTarget) < 25) {
                slidesPower = .2;
            }

            if (gamepad1.a && !aPressed) {
                clawClosed = !clawClosed;
            }
            aPressed = gamepad1.a;

            if (clawClosed) {
                claw1.setPosition(Constants.CLAW1_CLOSE);
                claw2.setPosition(Constants.CLAW2_CLOSE);
            } else {
                claw1.setPosition(Constants.CLAW1_OPEN);
                claw2.setPosition(Constants.CLAW2_OPEN);
            }

            slides.setTargetPosition(slidesTarget);
            slides.setPower(slidesPower);

            telemetry.addData("heading", Math.toDegrees(heading));
            telemetry.addData("slides target", slidesTarget);
            telemetry.addData("slides position", slides.getCurrentPosition());
            telemetry.addData("slides power", slidesPower);
            telemetry.addData("claw state", clawClosed ? "closed" : "open");
            telemetry.addData("slides current", "%f amps", slides.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }

    public double cube(double x) {
        return x*x*x;
    }
}
