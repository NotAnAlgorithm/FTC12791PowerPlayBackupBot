package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.subsystem.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Mecanum2;
import org.firstinspires.ftc.teamcode.subsystem.ServoEx;
import org.firstinspires.ftc.teamcode.subsystem.SimpleIMUIntegrator;
import org.firstinspires.ftc.teamcode.subsystem.Webcam;
import org.firstinspires.ftc.teamcode.subsystem.vision.SignalDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.AxisDirection;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

@Autonomous(preselectTeleOp = "MASTeleop")
public class MASAuton extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotorEx slides;
    private Servo claw;
    private ServoEx wrist;

    @Override
    public void runOpMode() throws InterruptedException {
//        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);
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

//        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu2");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new SimpleIMUIntegrator();
//        imu.initialize(parameters);
//        BNO055IMUUtil.remapZAxis(imu, AxisDirection.POS_X);

//        SignalDetectionPipeline pipeline = new SignalDetectionPipeline();
//        Webcam webcam = new Webcam(hardwareMap, pipeline);
//
//        SignalDetectionPipeline.ParkPosition parkPosition = SignalDetectionPipeline.ParkPosition.MIDDLE;

//        while (opModeInInit()) {
//            sleep(250);
//            claw.setPosition(Constants.CLAW1_CLOSE);
//            wrist.setPosition(Constants.CLAW2_CLOSE);
//            parkPosition = pipeline.position;
//            telemetry.addData("position", parkPosition);
//            if (pipeline.average != null) telemetry.addData("color", pipeline.average.toString());
//            telemetry.update();
//        }

        waitForStart();

        claw.setPosition(Constants.CLAW_CLOSE);
        sleep(1000);

        slides.setTargetPosition(300);
        slides.setPower(.5);

        double speed = .5;
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);
        sleep(1000);

        speed = 0;
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        slides.setTargetPosition(0);
        slides.setPower(.5);
        sleep(5000);
        slides.setPower(0);

        stop();
    }

}
