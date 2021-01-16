package org.firstinspires.ftc.teamcode.ops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.bots.TestBot;
import org.firstinspires.ftc.teamcode.components.DriveTrain;
import org.firstinspires.ftc.teamcode.util.Encoder;


@TeleOp(name = "BONAPARTE", group = "ops")
public class BONAPARTE extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private TestBot robot = null;
    private boolean logEnableTrace = false;
    private boolean logToTelemetry = true;



    public DcMotor Intake = null;
    public DcMotor Shooter1 = null;
    public DcMotor Shooter2 = null;
    public Servo ShootAngle = null;
    public Servo loader = null;
    public Servo indexer = null;
    public Servo wobbleArm = null;
    public Servo wobbleClaw = null;


    @Override
    public void runOpMode() {


        Intake = hardwareMap.dcMotor.get("rightIntake");
        Shooter1 = hardwareMap.dcMotor.get("shooterOne");
        Shooter2 = hardwareMap.dcMotor.get("shooterTwo");
        loader = hardwareMap.servo.get("loader");
        ShootAngle = hardwareMap.servo.get("shooterAngle");
        indexer = hardwareMap.servo.get("indexer");
        wobbleArm = hardwareMap.servo.get("wobbleArm");
        wobbleClaw = hardwareMap.servo.get("wobbleClaw");


        //Reverse spins motors to the right Forward spins motors to the left
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        Shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        Shooter2.setDirection(DcMotorSimple.Direction.FORWARD);


        robot = new TestBot(this, logEnableTrace, logToTelemetry);
        robot.logger.logInfo("runOpMode", "===== [ Start Initializing ]");

        /* Use either robot.initAll or select only the components that need initializing below */
        robot.initAll();
        robot.driveTrain.init(DriveTrain.InitType.INIT_4WD);
        robot.intake.init();

        robot.logger.logInfo("runOpMode", "===== [ Initialization Complete ]");
        telemetry.update();


        robot.driveTrain.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.driveTrain.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.driveTrain.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.driveTrain.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.logger.logInfo("runOpMode", "===== [ Start TeleOp ]");
        runtime.reset();


        while (opModeIsActive()) {

            //field oriented gamepad stuff
            FieldRelative(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

            //intake on and indexer to down postiion
            if (gamepad1.right_trigger > .1) {
                indexer.setPosition(.65);
                Intake.setPower(1);
            }

            //intake backwards
            else if (gamepad1.left_trigger > .1) {
                Intake.setPower(-1);
            }

            //intake off
            else {
                Intake.setPower(0);
            }

            //shooter on and indexer to up position
            if (gamepad1.left_bumper) {
                Shooter1.setPower(-.7);
                Shooter2.setPower(-.7);
                indexer.setPosition(1);
            }

            //shooter off
            else {
                Shooter1.setPower(0);
                Shooter2.setPower(0);
            }

            //loader out position
            if (gamepad1.right_bumper) {
                //change values
                loader.setPosition(.5);
            }

            //loader in position
            else {
                //change value
                loader.setPosition(.83);
            }

            //highgoal position
            if (gamepad1.dpad_down) {
                //change values
                ShootAngle.setPosition(.8);
            }

            //powershot position
            if (gamepad1.dpad_right) {
                //change values
                ShootAngle.setPosition(.78);
            }

            //highest position
            else if (gamepad1.dpad_up) {
                //change values
                ShootAngle.setPosition(1);
            }

            //wobble goal arm down
            if (gamepad1.x){
                //change values
                wobbleArm.setPosition(.45);
            }
            // wobble arm up
            else if (gamepad1.a) {
                //change values
                wobbleArm.setPosition(.05);
            }

            //wobble claw open
            if (gamepad1.y) {
                wobbleClaw.setPosition(.4);
            }

            //wobble claw close
            else if (gamepad1.b) {
                wobbleClaw.setPosition(.15);
            }

        }
    }

    /**
     * real gamer field relative driving
     *
     * @param ySpeed
     * @param xSpeed
     * @param turnSpeed
     **/
    public void FieldRelative(double ySpeed, double xSpeed, double turnSpeed) {
        double angle = robot.gyroNavigator.getAngleGood();
        angle = AngleWrapDeg(angle);
        angle = -angle;

        xSpeed = Range.clip(xSpeed, -1, 1);
        ySpeed = Range.clip(ySpeed, -1, 1);
        turnSpeed = Range.clip(turnSpeed, -1, 1);

        org.firstinspires.ftc.teamcode.geometry.Vector2d input = new org.firstinspires.ftc.teamcode.geometry.Vector2d(xSpeed, ySpeed);

        input = input.rotateBy(angle);

        double theta = input.angle();

        double frontLeftPower = input.magnitude() * Math.sin(theta + Math.PI / 4) + turnSpeed;
        double frontRightPower = input.magnitude() * Math.sin(theta - Math.PI / 4) - turnSpeed;
        double backLeftPower = input.magnitude() * Math.sin(theta - Math.PI / 4) + turnSpeed;
        double backRightPower = input.magnitude() * Math.sin(theta + Math.PI / 4) - turnSpeed;

        robot.driveTrain.backLeftMotor.setPower(backLeftPower);
        robot.driveTrain.backRightMotor.setPower(backRightPower);
        robot.driveTrain.frontLeftMotor.setPower(frontLeftPower);
        robot.driveTrain.frontRightMotor.setPower(frontRightPower);

        telemetry.addData("angle", angle);
        telemetry.addData("front encoder",robot.driveTrain.backLeftMotor.getCurrentPosition()*-1);
        telemetry.addData("left encoder",robot.driveTrain.frontLeftMotor.getCurrentPosition()*-1);
        telemetry.addData("right encoder",robot.driveTrain.backRightMotor.getCurrentPosition());


        telemetry.update();
    }


    public double AngleWrapDeg(double degrees) {
        while (degrees < -180) {
            degrees += 2.0 * 180;
        }
        while (degrees > 180) {
            degrees -= 2.0 * 180;
        }
        return degrees;
    }


}