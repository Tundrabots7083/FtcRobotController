package org.firstinspires.ftc.teamcode.ops;

import android.graphics.PostProcessor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.bots.TestBot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;


@Autonomous(name="bad", group="ops")
public class bad extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private TestBot robot = null;
    private boolean logEnableTrace = false;
    private boolean logToTelemetry = true;
    // OpenCV declaration
    OpenCvCamera webcam;
    SkystoneDeterminationPipeline pipeline;
    //shooter stuff
    private double SHOOTER_RPM = 7000;
    private double TICKS_PER_ROTATION = 14;
    private double FLYWHEEL_VELOCITY = (SHOOTER_RPM * TICKS_PER_ROTATION) / 60;
    //intake
    public DcMotor Intake = null;
    //servos
    public Servo wobbleArm;
    public Servo wobbleClaw;
    public DcMotor intake;


    private long LOADER_TIME = 300;

    private double AT_LINE_ANGLE = 0.87;
    private double BEHIND_STACK_ANGLE = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        Intake = hardwareMap.dcMotor.get("rightIntake");

        robot = new TestBot(this, logEnableTrace, logToTelemetry);

        robot.shooter.init();
        robot.loader.init();

        wobbleClaw = hardwareMap.get(Servo.class, "wobbleClaw");
        wobbleArm = hardwareMap.get(Servo.class, "wobbleArm");
        intake = hardwareMap.get(DcMotor.class, "rightIntake");

        wobbleClaw.setPosition(.15);
        wobbleArm.setPosition(0);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Dont look unless broken
        /*int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);*/

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        //      webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        // webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
       /* {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        }); */

        waitForStart();
        robot.shooter.init();
        robot.loader.init();


// uncomment while loop to tweak camera comment for robot to sense then do drive paths
           /* while (opModeIsActive()){
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position",pipeline.position);
            telemetry.update();
            sleep(50);
        }
        //sleeps for camera to have time to sense tweak to whatever is good
        sleep(1000);
*/

//------------------------------Drive-Paths-Below-------------------------------------------------\\
/*
      if (pipeline.getAnalysis()>pipeline.FOUR_RING_THRESHOLD) {

          //4 ring
          //set starting position
          drive.setPoseEstimate(new Pose2d(-63, -42, 0));
          wobbleArm.setPosition(.2);

          //move to behind the stack to shoot
          Trajectory move1 = drive.trajectoryBuilder(new Pose2d(-63, -42, 0))
                  .splineTo(new Vector2d(-38, -38), 0)
                  .build();

          drive.followTrajectory(move1);

          //shoot three rings
          robot.shooter.setShooterVelocity(FLYWHEEL_VELOCITY);
          robot.shooter.ShootAngle.setPosition(AT_LINE_ANGLE);
          robot.loader.indexer.setPosition(1);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.83);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.5);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.83);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.5);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.83);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.5);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.83);
          sleep(LOADER_TIME);
          robot.shooter.setShooterVelocity(0);
          robot.loader.indexer.setPosition(.71);
          intake.setPower(-1);

          //drive over stack to be epic
          Trajectory move2 = drive.trajectoryBuilder(move1.end())
                  .splineTo(new Vector2d(-27, -38), 0)
                  .build();

          drive.followTrajectory(move2);

          sleep(3000);

          //shoot three rings
          robot.shooter.setShooterVelocity(FLYWHEEL_VELOCITY);
          robot.shooter.ShootAngle.setPosition(AT_LINE_ANGLE);
          robot.loader.indexer.setPosition(1);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.83);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.5);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.83);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.5);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.83);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.5);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.83);
          sleep(LOADER_TIME);
          robot.shooter.setShooterVelocity(0);
          robot.loader.indexer.setPosition(.71);

          //drive to line to shoot again
          Trajectory move4 = drive.trajectoryBuilder(move2.end())
                  .splineTo(new Vector2d(0, -38), 0)
                  .build();

          drive.followTrajectory(move4);

          //shoot three rings
          robot.shooter.setShooterVelocity(FLYWHEEL_VELOCITY);
          robot.shooter.ShootAngle.setPosition(AT_LINE_ANGLE);
          robot.loader.indexer.setPosition(1);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.83);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.5);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.83);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.5);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.83);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.5);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.83);
          sleep(LOADER_TIME);
          robot.shooter.setShooterVelocity(0);
          intake.setPower(0);

          //drop off wobble 1
          Trajectory move5 = drive.trajectoryBuilder(move4.end())
                  .splineToLinearHeading(new Pose2d(49, -51, Math.toRadians(-25)), 0)
                  .build();

          drive.followTrajectory(move5);

          //drop wobble one
          wobbleArm.setPosition(.4);
          sleep(1000);
          wobbleClaw.setPosition(.35);
          sleep(200);
          wobbleArm.setPosition(.2);

          //go pickup wobble 2
          Trajectory move6 = drive.trajectoryBuilder(move5.end())
                  .splineToLinearHeading(new Pose2d(-30, -32, Math.toRadians(-180)), 0)
                  .build();

          drive.followTrajectory(move6);

          //pick up second wobble
          sleep(200);
          wobbleArm.setPosition(.5);
          sleep(500);
          wobbleClaw.setPosition(.15);
          sleep(500);
          wobbleArm.setPosition(.3);

          //drop off wobble 2
          Trajectory move7 = drive.trajectoryBuilder(move6.end())
                  .splineToLinearHeading(new Pose2d(41, -45, Math.toRadians(-15)), 0)
                  .build();

          drive.followTrajectory(move7);

          //drop second wobble
          sleep(200);
          wobbleArm.setPosition(.4);
          sleep(500);
          wobbleClaw.setPosition(.45);
          sleep(100);
          wobbleArm.setPosition(.2);
          sleep(100);
          wobbleClaw.setPosition(.2);
          sleep(100);
          wobbleArm.setPosition(0);
          sleep(100);

          //park on da line
          Trajectory move8 = drive.trajectoryBuilder(move7.end())
                  .splineTo(new Vector2d(10, -30), 0)
                  .build();

          drive.followTrajectory(move8);



        } else if (pipeline.getAnalysis()>pipeline.ONE_RING_THRESHOLD) {

          //1 ring
          //set starting position
          drive.setPoseEstimate(new Pose2d(-63, -42, 0));

          //move to behind the stack to shoot
          Trajectory move1 = drive.trajectoryBuilder(new Pose2d(-63, -42, 0))
                  .splineTo(new Vector2d(-38, -42), 0)
                  .build();

          drive.followTrajectory(move1);

          robot.shooter.setShooterVelocity(FLYWHEEL_VELOCITY);
          robot.shooter.ShootAngle.setPosition(AT_LINE_ANGLE);
          robot.loader.indexer.setPosition(1);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.83);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.5);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.83);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.5);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.83);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.5);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.83);
          sleep(LOADER_TIME);
          robot.loader.indexer.setPosition(.71);
          robot.intake.setIntakePower(1);
          robot.shooter.setShooterVelocity(0);

          //drive into the 1 ring
          Trajectory move2 = drive.trajectoryBuilder(move1.end())
                  .splineTo(new Vector2d(-30, -34), 0)
                  .build();

          drive.followTrajectory(move2);

          //shoot 1 ring
          robot.shooter.setShooterVelocity(FLYWHEEL_VELOCITY);
          robot.loader.indexer.setPosition(1);
          sleep(200);
          robot.loader.loaderServo.setPosition(.5);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.83);
          sleep(LOADER_TIME);
          robot.shooter.setShooterVelocity(0);

          //drive to zone b for wobble one
          Trajectory move3 = drive.trajectoryBuilder(move2.end())
                  .splineTo(new Vector2d(24, -32), 0)
                  .build();

          drive.followTrajectory(move3);

          //drop wobble one
          sleep(1000);
          //wobble arm down and release
          wobbleArm.setPosition(.4);
          sleep(500);
          wobbleClaw.setPosition(.4);
          sleep(200);
          wobbleArm.setPosition(.2);
          sleep(200);

          //drive to pick up second wobble
          Trajectory move4 = drive.trajectoryBuilder(move3.end())
                  .splineToLinearHeading(new Pose2d(-30, -32, Math.toRadians(180)), 0)
                  .build();

          drive.followTrajectory(move4);

          //pick up second wobble
          sleep(200);
          wobbleArm.setPosition(.5);
          sleep(1000);
          wobbleClaw.setPosition(.15);
          sleep(1000);
          wobbleArm.setPosition(.3);

          //drive to zone a to drop second wobble
          Trajectory move5 = drive.trajectoryBuilder(move4.end())
                  .splineToLinearHeading(new Pose2d(25, -32, Math.toRadians(0)), 0)
                  .build();

          drive.followTrajectory(move5);

          //drop second wobble
          sleep(200);
          wobbleArm.setPosition(.4);
          sleep(3000);
          wobbleClaw.setPosition(.45);
          sleep(1000);
          wobbleArm.setPosition(.2);
          sleep(500);
          wobbleClaw.setPosition(.2);
          sleep(500);
          wobbleArm.setPosition(0);
          sleep(500);

          //park on the launch line
          Trajectory move6 = drive.trajectoryBuilder(move5.end())
                  .splineToLinearHeading(new Pose2d(10, -30, Math.toRadians(0)), 0)
                  .build();

          drive.followTrajectory(move6);



      } else {

          //0 ring
          //set starting position
          drive.setPoseEstimate(new Pose2d(-63, -42, 0));

          //move to the launch line
          Trajectory move1 = drive.trajectoryBuilder(new Pose2d(-63, -42, 0))
                  .splineTo(new Vector2d(0, -42), 0)
                  .build();

          drive.followTrajectory(move1);

          //shoot three rings
          robot.shooter.setShooterVelocity(FLYWHEEL_VELOCITY);
          robot.shooter.ShootAngle.setPosition(AT_LINE_ANGLE);
          robot.loader.indexer.setPosition(1);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.83);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.5);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.83);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.5);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.83);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.5);
          sleep(LOADER_TIME);
          robot.loader.loaderServo.setPosition(.83);
          sleep(LOADER_TIME);
          robot.shooter.setShooterVelocity(0);

          //turn towards zone a
          Trajectory move2 = drive.trajectoryBuilder(move1.end())
                  .splineToLinearHeading(new Pose2d(-1, -52, Math.toRadians(-25)), 0)
                  .build();

          drive.followTrajectory(move2);

          //drop wobble one
          wobbleArm.setPosition(.4);
          sleep(1000);
          wobbleClaw.setPosition(.35);
          sleep(200);
          wobbleArm.setPosition(.2);


          //drive to pick up second wobble
          Trajectory move3 = drive.trajectoryBuilder(move2.end())
                  .splineToLinearHeading(new Pose2d(-30, -32, Math.toRadians(180)), 0)
                  .build();

          drive.followTrajectory(move3);

          //pick up second wobble
          sleep(200);
          wobbleArm.setPosition(.5);
          sleep(500);
          wobbleClaw.setPosition(.15);
          sleep(500);
          wobbleArm.setPosition(.3);

          //drive to zone a to drop second wobble
          Trajectory move4 = drive.trajectoryBuilder(move3.end())
                  .splineToLinearHeading(new Pose2d(-7, -45, Math.toRadians(-15)), 0)
                  .build();

          drive.followTrajectory(move4);

          //drop second wobble
          sleep(200);
          wobbleArm.setPosition(.4);
          sleep(500);
          wobbleClaw.setPosition(.45);
          sleep(100);
          wobbleArm.setPosition(.2);
          sleep(100);
          wobbleClaw.setPosition(.2);
          sleep(100);
          wobbleArm.setPosition(0);
          sleep(100);

          //park on the launch line
          Trajectory move5 = drive.trajectoryBuilder(move4.end())
                  .splineToLinearHeading(new Pose2d(10, -30, Math.toRadians(0)), 0)
                  .build();

          drive.followTrajectory(move5);



      }*/

        if (true) {
            //0 ring
            //set starting position
            drive.setPoseEstimate(new Pose2d(-63, -42, 0));

            //move to the launch line
            Trajectory move1 = drive.trajectoryBuilder(new Pose2d(-63, -42, 0))
                    .splineTo(new Vector2d(0, -42), 0)
                    .build();

            drive.followTrajectory(move1);

            //shoot three rings
            robot.shooter.setShooterVelocity(FLYWHEEL_VELOCITY);
            robot.shooter.ShootAngle.setPosition(AT_LINE_ANGLE);
            robot.loader.indexer.setPosition(1);
            sleep(LOADER_TIME);
            robot.loader.loaderServo.setPosition(.83);
            sleep(LOADER_TIME);
            robot.loader.loaderServo.setPosition(.5);
            sleep(LOADER_TIME);
            robot.loader.loaderServo.setPosition(.83);
            sleep(LOADER_TIME);
            robot.loader.loaderServo.setPosition(.5);
            sleep(LOADER_TIME);
            robot.loader.loaderServo.setPosition(.83);
            sleep(LOADER_TIME);
            robot.loader.loaderServo.setPosition(.5);
            sleep(LOADER_TIME);
            robot.loader.loaderServo.setPosition(.83);
            sleep(LOADER_TIME);
            robot.shooter.setShooterVelocity(0);

            //turn towards zone a
            Trajectory move2 = drive.trajectoryBuilder(move1.end())
                    .splineToLinearHeading(new Pose2d(-1, -52, Math.toRadians(-25)), 0)
                    .build();

            drive.followTrajectory(move2);

            //drop wobble one
            wobbleArm.setPosition(.4);
            sleep(1000);
            wobbleClaw.setPosition(.35);
            sleep(200);
            wobbleArm.setPosition(.2);


            //drive to pick up second wobble
            Trajectory move3 = drive.trajectoryBuilder(move2.end())
                    .splineToLinearHeading(new Pose2d(-30, -32, Math.toRadians(180)), 0)
                    .build();

            drive.followTrajectory(move3);

            //pick up second wobble
            sleep(200);
            wobbleArm.setPosition(.5);
            sleep(500);
            wobbleClaw.setPosition(.15);
            sleep(500);
            wobbleArm.setPosition(.3);

            //drive to zone a to drop second wobble
            Trajectory move4 = drive.trajectoryBuilder(move3.end())
                    .splineToLinearHeading(new Pose2d(-7, -45, Math.toRadians(-15)), 0)
                    .build();

            drive.followTrajectory(move4);

            //drop second wobble
            sleep(200);
            wobbleArm.setPosition(.4);
            sleep(500);
            wobbleClaw.setPosition(.45);
            sleep(100);
            wobbleArm.setPosition(.2);
            sleep(100);
            wobbleClaw.setPosition(.2);
            sleep(100);
            wobbleArm.setPosition(0);
            sleep(100);

            //park on the launch line
            Trajectory move5 = drive.trajectoryBuilder(move4.end())
                    .splineToLinearHeading(new Pose2d(10, -30, Math.toRadians(0)), 0)
                    .build();

            drive.followTrajectory(move5);


        }

    }
//------------------------------Methods-Below-Here------------------------------------------------\\

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }


        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        //location
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(93,92);
        //Size
        static final int REGION_WIDTH  = 35;
        static final int REGION_HEIGHT = 25;
        //Threshholds
        final int FOUR_RING_THRESHOLD = 160;
        final int ONE_RING_THRESHOLD  = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;

            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }

    }


    }

