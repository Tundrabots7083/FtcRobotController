package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TestPathing extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        //movement 1
        Trajectory movement1 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(-60, -45), 0)
                .build();

        drive.followTrajectory(movement1);

        /*

        //movement 2
        Trajectory movement2 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(-40, -30), 0)
                .build();

        drive.followTrajectory(movement2);

        //movement 3
        Trajectory movement3 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(-30, -35), 0)
                .build();

        drive.followTrajectory(movement3);

        //movement 4
        Trajectory movement4 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(50, -60), 0)
                .build();

        drive.followTrajectory(movement4);

        //movement 5
        Trajectory movement5 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(-40, -30), 180)
                .build();

        drive.followTrajectory(movement5);

        //movement 6
        Trajectory movement6 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(50, -60), 0)
                .build();

        drive.followTrajectory(movement6);

        //movement 7
        Trajectory movement7 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(10, -60), 0)
                .build();

        drive.followTrajectory(movement7);

        */

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
