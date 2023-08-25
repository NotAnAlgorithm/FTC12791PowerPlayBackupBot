package org.firstinspires.ftc.teamcode.opmode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.gamepad.JustPressed;
import org.firstinspires.ftc.teamcode.subsystem.Constants;

//@Disabled
@TeleOp
public class MASTeleop extends LinearOpMode{
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotorEx slides;
    private Servo claw;
    private Servo wrist;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        slides = hardwareMap.get(DcMotorEx.class, "slides");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setTargetPosition(0);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // slides.setTargetPositionTolerance(50);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        JustPressed justPressed1 = new JustPressed(gamepad1);

        int slidesTarget = Constants.SLIDES_DOWN;
        double slidesPower = 0;

        boolean clawClosed = false;
        boolean wristForward = true;

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

            frontLeft.setPower((rotY + rotX + turn) / denominator);
            frontRight.setPower((rotY - rotX - turn) / denominator);
            backLeft.setPower((rotY - rotX + turn) / denominator);
            backRight.setPower((rotY + rotX - turn) / denominator);

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

            if (justPressed1.a()) clawClosed = !clawClosed;
            if (justPressed1.b()) wristForward = !wristForward;

            claw.setPosition(clawClosed ? Constants.CLAW_CLOSE : Constants.CLAW_OPEN);
            wrist.setPosition(wristForward ? Constants.WRIST_FORWARD : Constants.WRIST_BACKWARD);

            slides.setTargetPosition(slidesTarget);
            slides.setPower(slidesPower);

            telemetry.addData("heading", Math.toDegrees(heading));
            telemetry.addData("slides target", slidesTarget);
            telemetry.addData("slides position", slides.getCurrentPosition());
            telemetry.addData("slides power", slidesPower);
            telemetry.addData("claw state", clawClosed ? "closed" : "open");
            telemetry.addData("wrist state", wristForward ? "forward" : "backward");
            telemetry.addData("slides current", "%f amps", slides.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
            justPressed1.update();
        }
    }

    public double cube(double x) {
        return x*x*x;
    }
}
