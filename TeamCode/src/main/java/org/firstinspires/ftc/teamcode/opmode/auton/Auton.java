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
import org.firstinspires.ftc.teamcode.subsystem.SimpleIMUIntegrator;
import org.firstinspires.ftc.teamcode.subsystem.Mecanum2;
import org.firstinspires.ftc.teamcode.subsystem.Webcam;
import org.firstinspires.ftc.teamcode.subsystem.vision.SignalDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.AxisDirection;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

@Autonomous(preselectTeleOp = "BackupTeleop")
public class Auton extends LinearOpMode {

    final double ENCODER_ADJUST = .61 / .53;

    @Override
    public void runOpMode() throws InterruptedException {
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        Mecanum2 drive = new Mecanum2(hardwareMap);
        drive.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        Deployment deployment = new Deployment(hardwareMap);

        Servo claw1 = hardwareMap.get(Servo.class, "claw1");
        Servo claw2 = hardwareMap.get(Servo.class, "claw2");
        
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new SimpleIMUIntegrator();
        imu.initialize(parameters);
        BNO055IMUUtil.remapZAxis(imu, AxisDirection.POS_X);

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
        double encoderDifference = 0;
        sleep(500);
        imu.startAccelerationIntegration(
                new Position(DistanceUnit.METER, 0, 0, 0, System.nanoTime()),
                new Velocity(DistanceUnit.METER, 0, 0, 0, System.nanoTime()),
                0
        );

        while (opModeInInit()) {
            sleep(250);
//            claw1.setPosition(Constants.CLAW1_CLOSE);
//            claw2.setPosition(Constants.CLAW2_CLOSE);
            parkPosition = pipeline.position;
            telemetry.addData("position", parkPosition);
            if (pipeline.average != null) telemetry.addData("color", pipeline.average.toString());
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            // all in meters
            Position position = imu.getPosition();
            Velocity velocity = imu.getVelocity();
            Acceleration acceleration = imu.getAcceleration();
            Orientation angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double encoder = drive.frontLeft.getCurrentPosition() / 537.7 * .096 * Math.PI * ENCODER_ADJUST - encoderDifference;

            if (state == State.FROM_START) {
                claw1.setPosition(Constants.CLAW1_CLOSE);
                claw2.setPosition(Constants.CLAW2_CLOSE);
                drive.setWeightedDrivePower(new Pose2d(-.5, 0, 0));
                if (Math.abs(encoder - .75) < .05) {
                    drive.setWeightedDrivePower(new Pose2d());
                    drive.update();
                    state = State.TO_TARGETS;
                    encoderDifference = encoder;
                    sleep(500);
                }
            } else if (state == State.TO_TARGETS) {
                switch (parkPosition) {
                    case LEFT:
                        drive.setWeightedDrivePower(new Pose2d(0, -.5, 0));
                        if (Math.abs(encoder + .75) < .05) state = State.DONE;
                        break;
                    case MIDDLE:
                        state = State.DONE;
                        break;
                    case RIGHT:
                        drive.setWeightedDrivePower(new Pose2d(0, .5, 0));
                        if (Math.abs(encoder - .79) < .05) state = State.DONE;
                        break;
                }
            } else if (state == State.DONE) {
                drive.setWeightedDrivePower(new Pose2d());
            }
            
            telemetry.addData("state", state);
            telemetry.addData("position", "%s m", position.toString());
            telemetry.addData("velocity", "%s m/s", velocity.toString());
            telemetry.addData("acceleration", "%s m/s/s", acceleration.toString());
            telemetry.addData("encoder", "%f m", encoder);
            telemetry.addData("angle", angle.firstAngle);
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.clearBulkCache();
            }
            drive.update();
            telemetry.update();
        }
    }

    public enum State {
        FROM_START,
        TO_TARGETS,
        DONE
    }
}
