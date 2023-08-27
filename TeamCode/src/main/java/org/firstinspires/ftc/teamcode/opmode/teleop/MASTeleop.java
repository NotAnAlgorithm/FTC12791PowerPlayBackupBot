package org.firstinspires.ftc.teamcode.opmode.teleop;


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
import org.firstinspires.ftc.teamcode.subsystem.ServoEx;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

//@Disabled
@TeleOp
public class MASTeleop extends LinearOpMode{
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotorEx slides;
    private Servo claw;
    private ServoEx wrist;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = TelemetryUtil.initTelemetry(telemetry);

        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        slides = hardwareMap.get(DcMotorEx.class, "slides");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = new ServoEx(hardwareMap.get(Servo.class, "wrist"));

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setTargetPosition(0);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // slides.setTargetPositionTolerance(50);

//        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        imu.initialize(parameters);

        JustPressed justPressed1 = new JustPressed(gamepad1);

        int slidesPosition = 0;
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
            if (gamepad1.right_bumper || slidesTarget > 2000) {
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
                slidesPower = .8;
            } else if (gamepad1.dpad_left) {
                slidesTarget = Constants.SLIDES_LOW;
                slidesPower = .75;
            } else if (gamepad1.dpad_right) {
                slidesTarget = Constants.SLIDES_MID;
                slidesPower = .75;
            } else if (gamepad1.dpad_up) {
                slidesTarget = Constants.SLIDES_HIGH;
                slidesPower = .9;
            } else if (gamepad1.left_bumper) {
                slidesTarget = Constants.SLIDES_GROUND;
                slidesPower = .65;
            }

            slidesPosition = slides.getCurrentPosition();
            slidesTarget += gamepad1.right_trigger * 10 - gamepad1.left_trigger * 8;

            if (Math.abs(slidesPosition - slidesTarget) < 40) {
                slidesPower = .3;
            } else {
                slidesPower = Math.max(slidesPower, .5);
            }

            if (justPressed1.a()) clawClosed = !clawClosed;
            if (justPressed1.b() && slidesPosition > Constants.SLIDES_LOW / 2) {
                wristForward = !wristForward;
                wrist.setPosition(wristForward ? Constants.WRIST_FORWARD : Constants.WRIST_BACKWARD, 700);
            }
            if (gamepad1.x) claw.setPosition((Constants.CLAW_CLOSE + Constants.CLAW_OPEN) / 2);

            claw.setPosition(clawClosed ? Constants.CLAW_CLOSE : Constants.CLAW_OPEN);

            slides.setTargetPosition(slidesTarget);
            slides.setPower(slidesPower);

            telemetry.addData("heading", Math.toDegrees(heading));
            telemetry.addData("slides target", slidesTarget);
            telemetry.addData("slides position", slidesPosition);
            telemetry.addData("slides power", slidesPower);
            telemetry.addData("claw state", clawClosed ? "closed" : "open");
            telemetry.addData("wrist state", wristForward ? "forward" : "backward");
            telemetry.addData("slides current", "%f amps", slides.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
            justPressed1.update();
            wrist.update();
        }
    }

    public double cube(double x) {
        return x*x*x;
    }
}
