package org.firstinspires.ftc.teamcode.ops;

        import com.acmerobotics.roadrunner.geometry.Pose2d;
        import com.acmerobotics.roadrunner.geometry.Vector2d;
        import com.acmerobotics.roadrunner.trajectory.Trajectory;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.util.ElapsedTime;

        import org.firstinspires.ftc.robotcore.internal.usb.exception.RobotUsbTimeoutException;
        import org.firstinspires.ftc.teamcode.bots.TestBot;
        import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
        import org.opencv.core.Core;
        import org.opencv.core.Mat;
        import org.opencv.core.Point;
        import org.opencv.core.Rect;
        import org.opencv.core.Scalar;
        import org.opencv.imgproc.Imgproc;
        import org.openftc.easyopencv.OpenCvPipeline;
        import com.qualcomm.hardware.bosch.BNO055IMU;
        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
        import org.openftc.easyopencv.OpenCvCamera;
        import org.openftc.easyopencv.OpenCvCameraFactory;
        import org.openftc.easyopencv.OpenCvCameraRotation;
        import org.firstinspires.ftc.teamcode.components.Shooter;
        import org.firstinspires.ftc.teamcode.components.Loader;



@Autonomous(name="auto", group="ops")
public class auto extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private TestBot robot = null;
    private boolean logEnableTrace = false;
    private boolean logToTelemetry = true;
    // OpenCV declaration
    OpenCvCamera webcam;
    SkystoneDeterminationPipeline pipeline;

    public DcMotor Intake = null;


    @Override
    public void runOpMode() throws InterruptedException {

        Intake = hardwareMap.dcMotor.get("rightIntake");

        robot = new TestBot(this, logEnableTrace, logToTelemetry);

        robot.shooter.init();
        robot.loader.init();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Dont look unless broken
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        //      webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        waitForStart();


// uncomment while loop to tweak camera comment for robot to sense then do drive paths
      /*  while (opModeIsActive()){
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position",pipeline.position);
            telemetry.update();
            sleep(50);
        } */
        //sleeps for camera to have time to sense tweak to whatever is good
        sleep(1000);


//------------------------------Drive-Paths-Below-------------------------------------------------\\

        if (pipeline.getAnalysis()>pipeline.FOUR_RING_THRESHOLD)
        {

/* Trajectory moveToShootPos = drive.trajectoryBuilder(new Pose2d())
                    .splineTo(new Vector2d(64, 0), 0)
                    .build();
            drive.followTrajectory(moveToShootPos);
        */
        }
        else if (pipeline.getAnalysis()>pipeline.ONE_RING_THRESHOLD)
        { }
        else
        {

             //set starting position
            drive.setPoseEstimate(new Pose2d(-63, -42, 0));

            //move to the launch line
            Trajectory move1 = drive.trajectoryBuilder(new Pose2d(0, -63, 0))
            .build();

            //turn on flywheel

            //angle shooter






            Trajectory traj = drive.trajectoryBuilder(new Pose2d(-63, -42, 0))
                    .splineTo(new Vector2d(-3, -36), 0)
                    .splineTo(new Vector2d(-3, -60), 0)
                    .splineTo(new Vector2d(-35, -32), 0)
                    .splineTo(new Vector2d(-3, -60), 0)
                    .splineTo(new Vector2d(10, -24), 0)
                    .addTemporalMarker(1, () -> {
                        robot.loader.indexer.setPosition(1);
                        robot.shooter.setShooterPower(.6);
                    })
                    .addTemporalMarker(1.4, () -> {
                        robot.loader.loaderServo.setPosition(.5);
                    })
                    .addTemporalMarker(1.8, () -> {
                        robot.loader.loaderServo.setPosition(.83);
                    })
                    .addTemporalMarker(2.2, () -> {
                        robot.loader.loaderServo.setPosition(.5);
                    })
                    .addTemporalMarker(2.6, () -> {
                        robot.loader.loaderServo.setPosition(.83);
                    })
                    .addTemporalMarker(3, () -> {
                        robot.loader.loaderServo.setPosition(.5);
                    })
                    .addTemporalMarker(3.4, () -> {
                        robot.loader.loaderServo.setPosition(.83);
                    })
                    .addTemporalMarker(3.8, () -> {
                        robot.shooter.setShooterPower(0);
                    })
                    .build();


/*
            Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(10)
                    .build();

            Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                    .forward(5)
                    .build();

            waitForStart();

            if(isStopRequested()) return;

            drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);
*/
            drive.setPoseEstimate(new Pose2d(-63, -42, 0));
//movement 1

         /*   Trajectory traj = drive.trajectoryBuilder(new Pose2d(-63, -42, 0))
                    .splineTo(new Vector2d(-3, -36), 0)
                    .splineTo(new Vector2d(-3, -60), 0)
                    .splineTo(new Vector2d(-35, -32), 0)
                    .splineTo(new Vector2d(-3, -60), 0)
                    .splineTo(new Vector2d(10, -24), 0)
                    .addTemporalMarker(1, () -> {
                        robot.loader.indexer.setPosition(1);
                        robot.shooter.setShooterPower(.6);
                    })
                    .addTemporalMarker(1.4, () -> {
                        robot.loader.loaderServo.setPosition(.5);
                    })
                    .addTemporalMarker(1.8, () -> {
                        robot.loader.loaderServo.setPosition(.83);
                    })
                    .addTemporalMarker(2.2, () -> {
                        robot.loader.loaderServo.setPosition(.5);
                    })
                    .addTemporalMarker(2.6, () -> {
                        robot.loader.loaderServo.setPosition(.83);
                    })
                    .addTemporalMarker(3, () -> {
                        robot.loader.loaderServo.setPosition(.5);
                    })
                    .addTemporalMarker(3.4, () -> {
                        robot.loader.loaderServo.setPosition(.83);
                    })
                    .addTemporalMarker(3.8, () -> {
                        robot.shooter.setShooterPower(0);
                    })
                    .build();

            drive.followTrajectory(traj); */

            /*robot.loader.indexer.setPosition(1);
            robot.shooter.setShooterPower(.6);
            robot.loader.loaderServo.setPosition(.5);
            robot.loader.loaderServo.setPosition(.83);
            robot.loader.loaderServo.setPosition(.5);
            robot.loader.loaderServo.setPosition(.83);
            robot.loader.loaderServo.setPosition(.5);
            robot.loader.loaderServo.setPosition(.83);
            robot.shooter.setShooterPower(0);*/
//shoot

            /*sleep(3000);

            //movement 2
            Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(-3, -36, 0))
                    .splineTo(new Vector2d(-3, -60), 0)
                    .build();
            drive.followTrajectory(traj1);

            sleep(3000);

            //movement 3

            Trajectory traj3 = drive.trajectoryBuilder(new Pose2d(-3, -60, 0))
                    .splineTo(new Vector2d(-35, -32), 0)
                    .build();
            drive.followTrajectory(traj3);

            sleep(3000);

            //movement 4

            Trajectory traj4 = drive.trajectoryBuilder(new Pose2d(-35, -32, 0))
                    .splineTo(new Vector2d(-3, -60), 0)
                    .build();
            drive.followTrajectory(traj4);

            sleep(3000);


            Trajectory traj5 = drive.trajectoryBuilder(new Pose2d(-3, -60, 0))
                    .splineTo(new Vector2d(10, -24), 0)
                    .build();
            drive.followTrajectory(traj5);

            sleep(3000);

            Intake.setPower(1);*/
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

