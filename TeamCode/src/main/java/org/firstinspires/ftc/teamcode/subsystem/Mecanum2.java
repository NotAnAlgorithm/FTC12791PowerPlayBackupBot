package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Mecanum2 {
    public DcMotorEx frontLeft, backLeft, backRight, frontRight;
    public MultiMotor motors;
    
    private double[] wheelVelocities = new double[4];

    public Mecanum2(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");

        motors = new MultiMotor(frontLeft, backLeft, backRight, frontRight);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update() {
        frontLeft.setPower(wheelVelocities[0]);
        backLeft.setPower(wheelVelocities[1]);
        backRight.setPower(wheelVelocities[2]);
        frontRight.setPower(wheelVelocities[3]);
    }

    /**
     * Drives the robot using field-centric directions given robot-centric inputs.
     * This assumes that the robot's 0 degrees is the same as the field's 0 degrees.
     * @param pose Robot-centric directions.
     * @param currentHeading Robot's current heading.
     */
    public void driveFieldCentric(Pose2d pose, double currentHeading) {
        Vector2d rotated = new Vector2d(pose.getX(), pose.getY()).rotated(currentHeading);
        setWeightedDrivePower(new Pose2d(rotated, pose.getHeading()));
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        double denom = Math.max(Math.abs(drivePower.getX()) + Math.abs(drivePower.getY()) + Math.abs(drivePower.getHeading()), 1);
        Pose2d vel = drivePower.div(denom);

        wheelVelocities = new double[] {
                vel.getX() - vel.getY() - vel.getHeading(),
                vel.getX() + vel.getY() - vel.getHeading(),
                vel.getX() - vel.getY() + vel.getHeading(),
                vel.getX() + vel.getY() + vel.getHeading()
        };
    }
}
