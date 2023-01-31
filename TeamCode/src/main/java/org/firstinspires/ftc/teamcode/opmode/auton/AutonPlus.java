package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
import org.firstinspires.ftc.teamcode.subsystem.SimpleIMUIntegrator;
import org.firstinspires.ftc.teamcode.subsystem.Webcam;
import org.firstinspires.ftc.teamcode.subsystem.vision.SignalDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

@Config
@Autonomous(preselectTeleOp = "BackupTeleop")
public class AutonPlus extends LinearOpMode {

    final double ENCODER_ADJUST = .61 / .53;

    public static double angleProportional = 2.0;
    public static double startDistance = 1.45;
    public static double deployDistance = 1.19;
    public static double deployCloseDistance = .2;
    public static double deployCloseDistance2 = .15;
    public static double signalLeftDistance = .3;
    public static double signalMidDistance = 1;
    public static double signalRightDistance = 1.7;

    @Override
    public void runOpMode() throws InterruptedException {
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        Mecanum2 drive = new Mecanum2(hardwareMap);
        drive.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        Deployment deployment = new Deployment(hardwareMap);

        DcMotorEx slides = hardwareMap.get(DcMotorEx.class, "slides");
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setTargetPosition(0);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Servo claw1 = hardwareMap.get(Servo.class, "claw1");
        Servo claw2 = hardwareMap.get(Servo.class, "claw2");

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new SimpleIMUIntegrator();
        imu.initialize(parameters);
//        BNO055IMUUtil.remapZAxis(imu, AxisDirection.POS_X);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        for (DcMotorEx motor : drive.motors.motors) {
            MotorConfigurationType configuration = motor.getMotorType().clone();
            configuration.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(configuration);
        }

        SignalDetectionPipeline pipeline = new SignalDetectionPipeline();
        Webcam webcam = new Webcam(hardwareMap, pipeline);

        SignalDetectionPipeline.ParkPosition parkPosition = SignalDetectionPipeline.ParkPosition.MIDDLE;
        PIDFController pid = new PIDFController(new PIDCoefficients(2, 0, 0));
        State state = State.FROM_START;
        double encoderDifference1 = 0;
        double encoderDifference2 = 0;
        int slidesTarget = Constants.SLIDES_DOWN + 100;
        double slidesPower = .4;
        double angleTarget = 0;
        sleep(500);
        imu.startAccelerationIntegration(
                new Position(DistanceUnit.METER, 0, 0, 0, System.nanoTime()),
                new Velocity(DistanceUnit.METER, 0, 0, 0, System.nanoTime()),
                0
        );

        while (opModeInInit()) {
            sleep(250);
            claw1.setPosition(Constants.CLAW1_CLOSE);
            claw2.setPosition(Constants.CLAW2_CLOSE);
            parkPosition = pipeline.position;
            telemetry.addData("position", parkPosition);
            if (pipeline.average != null) telemetry.addData("color", pipeline.average.toString());
            telemetry.update();
        }

        webcam.close();
        waitForStart();

        while (opModeIsActive()) {
            // all in meters
            Position position = imu.getPosition();
            Velocity velocity = imu.getVelocity();
            Acceleration acceleration = imu.getAcceleration();
            Orientation angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double encoder1 = drive.frontLeft.getCurrentPosition() / 537.7 * .096 * Math.PI * ENCODER_ADJUST - encoderDifference1;
            double encoder2 = drive.backRight.getCurrentPosition() / 537.7 * .096 * Math.PI * ENCODER_ADJUST - encoderDifference2;
            double encoderSum = .5 * (encoder1 + encoder2);
            double encoderDif = .5 * (encoder1 - encoder2);

            if (state == State.FROM_START) {
                claw1.setPosition(Constants.CLAW1_CLOSE);
                claw2.setPosition(Constants.CLAW2_CLOSE);
                drive.setWeightedDrivePower(new Pose2d(-.35, 0, (angleTarget - angle.firstAngle) * angleProportional));
                if (Math.abs(encoderSum - startDistance) < .05) {
                    drive.setWeightedDrivePower(new Pose2d());
                    drive.update();
                    state = State.DEPLOY_PRELOAD_MOVE;
                    encoderDifference1 = encoder1;
                    encoderDifference2 = encoder2;
                    sleep(500);
                }
//            } else if (state == State.DEPLOY_PRELOAD_TURN) {
//                angleTarget = -Math.PI / 2;
//                drive.setWeightedDrivePower(new Pose2d(0, 0, -.2 + (angleTarget - angle.firstAngle) * angleProportional));
//                if (Math.abs(angle.firstAngle - angleTarget) < .05) {
//                    drive.setWeightedDrivePower(new Pose2d());
//                    drive.update();
//                    state = State.DEPLOY_PRELOAD_MOVE;
//                    encoderDifference = encoderSum;
//                    sleep(300);
//                }
            } else if (state == State.DEPLOY_PRELOAD_MOVE) {
                drive.setWeightedDrivePower(new Pose2d(0, -.35, (angleTarget - angle.firstAngle) * angleProportional));
                if (Math.abs(encoderDif + deployDistance) < .05) {
                    drive.setWeightedDrivePower(new Pose2d());
                    drive.update();
                    state = State.DEPLOY_PRELOAD_LIFT;
                    encoderDifference1 = encoder1;
                    encoderDifference2 = encoder2;
                    sleep(900);
                }
            } else if (state == State.DEPLOY_PRELOAD_LIFT) {
                slidesTarget = Constants.SLIDES_HIGH;
                slidesPower = .7;
                if (Math.abs(slides.getCurrentPosition() - slidesTarget) < 25) {
                    state = State.DEPLOY_PRELOAD;
                    encoderDifference1 = encoder1;
                    encoderDifference2 = encoder2;
                }
            } else if (state == State.DEPLOY_PRELOAD) {
                slidesPower = .2;
                drive.setWeightedDrivePower(new Pose2d(.15, 0, (angleTarget - angle.firstAngle) * angleProportional));
                if (Math.abs(encoderSum + deployCloseDistance) < .02) {
                    drive.setWeightedDrivePower(new Pose2d());
                    drive.update();
                    state = State.DEPLOY_DONE;
                    encoderDifference1 = encoder1;
                    encoderDifference2 = encoder2;
                    sleep(1000);
                }
            } else if (state == State.DEPLOY_DONE) {
                slidesTarget = Constants.SLIDES_DOWN;
                slidesPower = .65;
                if (Math.abs(slides.getCurrentPosition() - slidesTarget) < 100) {
                    state = State.DEPLOY_PRELOAD_LEAVE;
                    encoderDifference1 = encoder1;
                    encoderDifference2 = encoder2;
                }
            } else if (state == State.DEPLOY_PRELOAD_LEAVE) {
                claw1.setPosition(Constants.CLAW1_OPEN);
                claw2.setPosition(Constants.CLAW2_OPEN);
                drive.setWeightedDrivePower(new Pose2d(-.1, 0, (angleTarget - angle.firstAngle) * angleProportional));
                if (Math.abs(encoderSum - deployCloseDistance2) < .04) {
                    drive.setWeightedDrivePower(new Pose2d());
                    state = State.TO_SIGNAL_ZONE;
                    encoderDifference1 = encoder1;
                    encoderDifference2 = encoder2;
                }
            } else if (state == State.TO_SIGNAL_ZONE) {
                double distance = 0;
                switch (parkPosition) {
                    case LEFT: distance = signalLeftDistance; break;
                    case MIDDLE: distance = signalMidDistance; break;
                    case RIGHT: distance = signalRightDistance; break;
                }
                drive.setWeightedDrivePower(new Pose2d(0, .4, (angleTarget - angle.firstAngle) * angleProportional));
                if (Math.abs(encoderDif - distance) < .05) state = State.DONE;
            } else if (state == State.DONE) {
                slidesPower = .2;
                slidesTarget = Constants.SLIDES_DOWN;
                drive.setWeightedDrivePower(new Pose2d());
            }

            telemetry.addData("state", state);
            telemetry.addData("position", "%s m", position.toString());
            telemetry.addData("velocity", "%s m/s", velocity.toString());
            telemetry.addData("acceleration", "%s m/s/s", acceleration.toString());
            telemetry.addData("encoderSum", "%f m", encoderSum);
            telemetry.addData("encoderDif", "%f m", encoderDif);
            telemetry.addData("angle", angle.firstAngle);
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.clearBulkCache();
            }
            drive.update();
            slides.setTargetPosition(slidesTarget);
            slides.setPower(slidesPower);
            telemetry.update();
        }
    }

    public enum State {
        FROM_START,
        DEPLOY_PRELOAD_TURN,
        DEPLOY_PRELOAD_MOVE,
        DEPLOY_PRELOAD_LIFT,
        DEPLOY_PRELOAD,
        DEPLOY_PRELOAD_LEAVE,
        DEPLOY_DONE,
        BACK_TO_ZONE,
        TO_SIGNAL_ZONE,
        DONE
    }
}
