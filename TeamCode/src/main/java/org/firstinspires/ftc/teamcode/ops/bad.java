package org.firstinspires.ftc.teamcode.ops;

import android.graphics.PostProcessor;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.TuningController;
import org.firstinspires.ftc.teamcode.VelocityPIDFController;
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

@Autonomous(name = "bad", group = "ops")
public class bad extends LinearOpMode {
    //Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private TestBot robot = null;
    private boolean logEnableTrace = false;
    private boolean logToTelemetry = true;
    //OpenCV declaration
    OpenCvCamera webcam;
    SkystoneDeterminationPipeline pipeline;

    //shooter pid nonsense
    // Copy your PID Coefficients here
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.0016, 0, 0.0000016);

    // Copy your feedforward gains here
    public static double kV = 0.000435 / TuningController.rpmToTicksPerSecond(TuningController.MOTOR_MAX_RPM);
    public static double kA = 0.00015;
    public static double kStatic = 0;

    // Timer for calculating desired acceleration
    // Necessary for kA to have an affect
    private final ElapsedTime veloTimer = new ElapsedTime();
    private double lastTargetVelo = 0.0;

    // Our velocity controller
    private final VelocityPIDFController veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);

    //end of pid stuff idk

    //shooter
   /*
    private double SHOOTER_RPM = 6000;
    private double TICKS_PER_ROTATION = 14;
    private double FLYWHEEL_VELOCITY = (SHOOTER_RPM * TICKS_PER_ROTATION) / 60;

   */
    //I THINK I CAN COMMENT THIS STUFF OUT BECAUSE ITS DOWN THERE SOMEWHERE IDK

    //motors
    public DcMotor intake = null;

    //servos
    public Servo wobbleArm;
    public Servo wobbleClaw;
    //variables
    private long LOADER_TIME = 300;
    private double INDEXER_UP = 1;
    private double INDEXER_DOWN = 0.65;
    private double ANGLE_A = 0.85; // at shooting line
    private double ANGLE_B = 0.85; //halfway
    private double ANGLE_C = 0.84; //behind the stack

    DcMotorEx myMotor1;
    DcMotorEx myMotor2;
    double power;


    @Override
    public void runOpMode() throws InterruptedException {

        //morepid
        // SETUP MOTORS //
        // Change my id
        myMotor1 = hardwareMap.get(DcMotorEx.class, "shooterOne");
        myMotor2 = hardwareMap.get(DcMotorEx.class, "shooterTwo");

        // Reverse as appropriate
        //myMotor1.setDirection(DcMotor.Direction.REVERSE);
        // myMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Ensure that RUN_USING_ENCODER is not set
        myMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Turns on bulk reading
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        //more pid


        wobbleClaw = hardwareMap.get(Servo.class, "wobbleClaw");
        wobbleArm = hardwareMap.get(Servo.class, "wobbleArm");
        intake = hardwareMap.get(DcMotor.class, "rightIntake");


        robot = new TestBot(this, logEnableTrace, logToTelemetry);

        robot.shooter.init();
        robot.loader.init();

        wobbleClaw.setPosition(.15);
        wobbleArm.setPosition(0);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Dont look unless broken
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        waitForStart();
        robot.shooter.init();
        robot.loader.init();


        // uncomment while loop to tweak camera comment for robot to sense then do drive paths
            /*while (opModeIsActive())
            {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position",pipeline.position);
            telemetry.update();
            sleep(50);
            }*/
        //sleeps for camera to have time to sense tweak to whatever is good
        sleep(1000);

        //PID STUFF AAAAA


        ///// Run the velocity controller ////

        // Target velocity in ticks per second

        double SHOOTER_RPM = 4750;
        double TICKS_PER_ROTATION = 28 * (3 / 2);
        double FLYWHEEL_VELOCITY = (SHOOTER_RPM * TICKS_PER_ROTATION) / 60;
        double targetVelo = FLYWHEEL_VELOCITY;

        // Call necessary controller methods
        veloController.setTargetVelocity(targetVelo);
        veloController.setTargetAcceleration((targetVelo - lastTargetVelo) / veloTimer.seconds());
        veloTimer.reset();

        lastTargetVelo = targetVelo;

        // Get the velocity from the motor with the encoder
        double motorPos = myMotor1.getCurrentPosition();
        double motorVelo = myMotor1.getVelocity();

        // Update the controller and set the power for each motor
        double power = veloController.update(motorPos, motorVelo);

        //PID STUFF AAAAAAAAAAAA IDK WHAT IM DOING


//------------------------------Drive-Paths-Below-------------------------------------------------\\
        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);
        telemetry.update();
        wobbleArm.setPosition(.2);

        if (pipeline.getAnalysis() > pipeline.FOUR_RING_THRESHOLD) {
            //4 ring
            webcam.stopStreaming();

            //set starting position
            drive.setPoseEstimate(new Pose2d(-63, -43, 0));

            //move to behind the stack to shoot
            Trajectory move1 = drive.trajectoryBuilder(new Pose2d(-63, -43, 0))
                    .lineToLinearHeading(new Pose2d(-38, -38, 0))
                    .build();

            drive.followTrajectory(move1);

            //shoot three rings
            Shoot(3, ANGLE_C);

            //intake on
            intake.setPower(-1);

            //drive over stack to be epic
            Trajectory move2 = drive.trajectoryBuilder(move1.end())
                    .splineTo(new Vector2d(-27, -38), 0)
                    .build();

            drive.followTrajectory(move2);

            sleep(1500);

            //shoot three rings
            Shoot(3, ANGLE_B);

            //drive to line to shoot again
            Trajectory move4 = drive.trajectoryBuilder(move2.end())
                    .splineTo(new Vector2d(-4, -38), 0)
                    .build();

            drive.followTrajectory(move4);

            sleep(3000);

            //shoot three rings
            Shoot(3, ANGLE_A);

            //intake off
            intake.setPower(0);

            //drop off wobble 1
            Trajectory move5 = drive.trajectoryBuilder(move4.end())
                    .lineToLinearHeading(new Pose2d(49, -51, Math.toRadians(-25)))
                    .build();

            drive.followTrajectory(move5);

            //drop wobble one
            ReleaseWobble();

            //go pickup wobble 2
            Trajectory move6 = drive.trajectoryBuilder(move5.end())
                    .lineToLinearHeading(new Pose2d(-31, -29, Math.toRadians(180)))
                    .build();

            drive.followTrajectory(move6);

            //pick up second wobble
            PickupWobble();

            //drop off wobble 2
            Trajectory move7 = drive.trajectoryBuilder(move6.end())
                    .lineToLinearHeading(new Pose2d(40, -41, Math.toRadians(-17)))
                    .build();

            drive.followTrajectory(move7);

            //yeet wobble two
            ReleaseWobble();

            Trajectory move69 = drive.trajectoryBuilder(move7.end())
                    .lineToSplineHeading(new Pose2d(10, -30, 0))
                    .build();

            drive.followTrajectory(move69);



            /*
            //park on da line
            Trajectory move8 = drive.trajectoryBuilder(move7.end())
                    .splineToLinearHeading(new Pose2d(10, -30, 0), 0)
                    .build();

            drive.followTrajectory(move8);
             */

        } else if (pipeline.getAnalysis() > pipeline.ONE_RING_THRESHOLD) {
            //1 ring
            webcam.stopStreaming();

            //set starting position
            drive.setPoseEstimate(new Pose2d(-63, -43, 0));

            //move to behind the stack to shoot
            Trajectory move1 = drive.trajectoryBuilder(new Pose2d(-63, -43, 0))
                    .lineToLinearHeading(new Pose2d(-38, -38, 0))
                    .build();

            drive.followTrajectory(move1);

            //shoot 3 rings
            Shoot(3, ANGLE_C);

            //intake on
            intake.setPower(-1);

            //drive into the 1 ring
            Trajectory move2 = drive.trajectoryBuilder(move1.end())
                    .splineTo(new Vector2d(-30, -38), 0)
                    .build();

            drive.followTrajectory(move2);

            //move to the launch line
            Trajectory move69 = drive.trajectoryBuilder(move2.end())
                    .splineTo(new Vector2d(-4, -38), 0)
                    .build();

            drive.followTrajectory(move2);

            sleep(1000);

            //shoot 1 ring
            Shoot(1, ANGLE_B);

            //intake off
            intake.setPower(0);

            //drive to zone b for wobble one
            Trajectory move3 = drive.trajectoryBuilder(move69.end())
                    .splineTo(new Vector2d(17.5, -26), 0)
                    .build();

            drive.followTrajectory(move3);

            //drop wobble one
            ReleaseWobble();

            //drive to pick up second wobble
            Trajectory move4 = drive.trajectoryBuilder(move3.end())
                    .splineToLinearHeading(new Pose2d(-33, -30, Math.toRadians(180)), 0)
                    .build();

            drive.followTrajectory(move4);

            //pick up second wobble
            PickupWobble(500);

            //drive to zone a to drop second wobble
            Trajectory move5 = drive.trajectoryBuilder(move4.end())
                    .splineToLinearHeading(new Pose2d(12.5, -26, Math.toRadians(0)), 0)
                    .build();

            drive.followTrajectory(move5);

            //drop second wobble
            ReleaseWobble();

            //park on the launch line
            Trajectory move6 = drive.trajectoryBuilder(move5.end())
                    .splineToLinearHeading(new Pose2d(10, -30, Math.toRadians(0)), 0)
                    .build();

            drive.followTrajectory(move6);

        } else {
            //0 ring
            webcam.stopStreaming();

            //set starting position
            drive.setPoseEstimate(new Pose2d(-63, -43, 0));

            //move to the launch line
            Trajectory move1 = drive.trajectoryBuilder(new Pose2d(-63, -43, 0))
                    .splineTo(new Vector2d(-4, -42), 0)
                    .build();

            drive.followTrajectory(move1);

            //shoot three rings
            Shoot(3, ANGLE_A);

            //turn towards zone a
            Trajectory move2 = drive.trajectoryBuilder(move1.end())
                    .splineToLinearHeading(new Pose2d(-1, -52, Math.toRadians(-25)), 0)
                    .build();

            drive.followTrajectory(move2);

            //drop wobble one
            ReleaseWobble();

            //drive to pick up second wobble
            Trajectory move3 = drive.trajectoryBuilder(move2.end())
                    .splineToLinearHeading(new Pose2d(-31, -29, Math.toRadians(180)), 0)
                    .build();

            drive.followTrajectory(move3);

            //pick up second wobble
            PickupWobble();

            //drive to zone a to drop second wobble
            Trajectory move4 = drive.trajectoryBuilder(move3.end())
                    .splineToLinearHeading(new Pose2d(-7, -45, Math.toRadians(-15)), 0)
                    .build();

            drive.followTrajectory(move4);

            ReleaseWobble();

            //park on the launch line
            Trajectory move5 = drive.trajectoryBuilder(move4.end())
                    .splineToLinearHeading(new Pose2d(10, -30, Math.toRadians(0)), 0)
                    .build();

            drive.followTrajectory(move5);

        }

    }
//------------------------------Methods-Below-Here------------------------------------------------\\

    /**
     * Shoot
     *
     * @param Shots
     * @param ShooterAngle
     */
    public void Shoot(double Shots, double ShooterAngle) {
        //flywheel on
        myMotor1.setPower(power);
        myMotor2.setPower(power);
        //shooter angle
        robot.shooter.ShootAngle.setPosition(ShooterAngle);
        //indexer up
        robot.loader.indexer.setPosition(INDEXER_UP);
        //wait for shooter to get to speed
        sleep(500);
        //load rings
        for (double i = 0; i < Shots; i++) {
            robot.loader.loaderServo.setPosition(.83);
            sleep(LOADER_TIME);
            robot.loader.loaderServo.setPosition(.5);
            sleep(LOADER_TIME);
        }
        //loader arm in position
        robot.loader.loaderServo.setPosition(.83);
        //flywheel off
        myMotor1.setPower(0);
        myMotor2.setPower(0);
        //indexer down
        robot.loader.indexer.setPosition(INDEXER_DOWN);
    }

    /**
     * shooting method that doesn't work yet
     *
     * @param Shots
     * @param ShooterAngle
     */
   /* public void bad(double Shots, double ShooterAngle) {
        //flywheel on
        robot.shooter.setShooterVelocity(FLYWHEEL_VELOCITY);
        //shooter angle
        robot.shooter.ShootAngle.setPosition(ShooterAngle);
        //indexer up
        robot.loader.indexer.setPosition(INDEXER_UP);
        //wait for shooter to get to speed
        while (robot.shooter.getShooterVelocity() > FLYWHEEL_VELOCITY - 5) {

            // do nothing and wait

        }
        //load rings
        for (double i = 0; i < Shots; i++) {
            robot.loader.loaderServo.setPosition(.83);
            sleep(LOADER_TIME);
            robot.loader.loaderServo.setPosition(.5);
            sleep(LOADER_TIME);
        }
        //loader arm in position
        robot.loader.loaderServo.setPosition(.83);
        //flywheel off
        robot.shooter.setShooterVelocity(0);
        //indexer down
        robot.loader.indexer.setPosition(INDEXER_DOWN);
    }
*/

    /**
     * ReleaseWobble
     */
    public void ReleaseWobble() {
        wobbleArm.setPosition(.4);
        sleep(750);
        wobbleClaw.setPosition(.43);
        sleep(200);
        wobbleArm.setPosition(.2);
    }

    /**
     * PickupWobble
     */
    public void PickupWobble() {
        wobbleArm.setPosition(.45);
        sleep(500);
        wobbleClaw.setPosition(.15);
        sleep(200);
        wobbleArm.setPosition(.2);
        sleep(200);
    }

    /**
     * slow boi
     *
     * @param sleepTime
     */
    public void PickupWobble(long sleepTime) {
        wobbleArm.setPosition(.45);
        sleep(500);
        wobbleClaw.setPosition(.15);
        sleep(sleepTime);
        wobbleArm.setPosition(.2);
        sleep(200);
    }

    //OpenCV Methods
    public static class SkystoneDeterminationPipeline extends OpenCvPipeline {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition {
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
        static final Point REGION1_TOP_LEFT_ANCHOR_POINT = new Point(158, 155);
        //Size
        static final int REGION_WIDTH = 45;
        static final int REGION_HEIGHT = 30;
        //Threshholds
        final int FOUR_RING_THRESHOLD = 153;
        final int ONE_RING_THRESHOLD = 137;

        Point region1_pointA = new Point(
                REGION1_TOP_LEFT_ANCHOR_POINT.x,
                REGION1_TOP_LEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOP_LEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOP_LEFT_ANCHOR_POINT.y + REGION_HEIGHT);

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
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if (avg1 > FOUR_RING_THRESHOLD) {
                position = RingPosition.FOUR;

            } else if (avg1 > ONE_RING_THRESHOLD) {
                position = RingPosition.ONE;
            } else {
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis() {
            return avg1;
        }

    }


}

