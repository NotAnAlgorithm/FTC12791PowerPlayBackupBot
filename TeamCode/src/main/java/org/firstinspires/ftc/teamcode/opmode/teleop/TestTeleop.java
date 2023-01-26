package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Robot;

//@Disabled
@TeleOp
public class TestTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
//            Pose2d poseEstimate = robot.drive.getPoseEstimate();

            robot.drive.setWeightedDrivePower(new Pose2d(
                    cube(gamepad1.left_stick_x),
                    cube(-gamepad1.left_stick_y),
                    cube(gamepad1.right_stick_x)
            ));

//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("heading", robot.getHeading());
            robot.update();
        }
    }

    public static double cube(double n) {
        return n*n*n;
    }
}
