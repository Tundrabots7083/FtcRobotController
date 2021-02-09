package org.firstinspires.ftc.teamcode.ops;

import android.net.wifi.hotspot2.omadm.PpsMoParser;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.IncludedFirmwareFileInfo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.bots.TestBot;
import org.firstinspires.ftc.teamcode.components.DriveTrain;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.Loader;
import org.firstinspires.ftc.teamcode.components.Shooter;
import org.firstinspires.ftc.teamcode.geometry.Vector2d;



@TeleOp(name="bonapartebutpowerful", group="ops")
public class BONAPARTpower extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private TestBot robot = null;
    private boolean logEnableTrace = false;
    private boolean logToTelemetry = true;

    private double SHOOTER_RPM = 7000;
    private double TICKS_PER_ROTATION = 14;
    private double FLYWHEEL_VELOCITY = (SHOOTER_RPM * TICKS_PER_ROTATION) / 60;

    public Servo wobbleArm;
    public Servo wobbleClaw;



    @Override
    public void runOpMode() {

        robot = new TestBot(this, logEnableTrace, logToTelemetry);
        robot.logger.logInfo("runOpMode", "===== [ Start Initializing ]");

        /* Use either robot.initAll or select only the components that need initializing below */

        robot.logger.logInfo("runOpMode", "===== [ Initialization Complete ]");
        telemetry.update();

        wobbleClaw = hardwareMap.get(Servo.class, "wobbleClaw");
        wobbleArm = hardwareMap.get(Servo.class, "wobbleArm");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.shooter.init();
        robot.loader.init();
        robot.intake.init();
        robot.driveTrain.init(DriveTrain.InitType.INIT_4WD);
        robot.gyroNavigator.init();

        robot.logger.logInfo("runOpMode", "===== [ Start TeleOp ]");
        runtime.reset();

        wobbleArm.setPosition(.2);
        robot.shooter.ShootAngle.setPosition(.85);

        while (opModeIsActive()) {

            //field oriented gamepad stuff
            FieldRelative(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

            //sketchy intake code idk if work
            if (gamepad1.right_trigger > .1) {
                robot.loader.indexer.setPosition(.71);
                robot.intake.setIntakePower(1);
            }
            else if (gamepad1.left_trigger  > .1)
            {
                robot.intake.setIntakePower(-1);
            }
            else
            {
                robot.intake.setIntakePower(0);
            }

            if (gamepad1.left_bumper)
            {
                robot.shooter.setShooterPower(.7);
                robot.shooter.setShooterVelocity(FLYWHEEL_VELOCITY);
                robot.loader.indexer.setPosition(1);
            }
            else
            {
                robot.shooter.setShooterVelocity(0);
            }

            if (gamepad1.right_bumper)
            {
                //change values
                robot.loader.loaderServo.setPosition(.5);
            }
            else
            {
                //change value
                robot.loader.loaderServo.setPosition(.83);
            }

            if      (gamepad1.dpad_down)
            {
                //change values
                robot.shooter.ShootAngle.setPosition(.85);
            }
            else if (gamepad1.dpad_up)
            {
                //change values
                robot.shooter.ShootAngle.setPosition(.83);
            }

            if     (gamepad1.y)
            {
                wobbleArm.setPosition(.4);
                sleep(200);
                wobbleClaw.setPosition(.45);
            }
            else if(gamepad1.b)
            {
                wobbleClaw.setPosition(0.15);
                sleep(200);
                wobbleArm.setPosition(0);
            }

            if(gamepad1.)
            telemetry.addData("Shooter velocity: ",robot.shooter.getShooterVelocity());
            telemetry.update();
        }


    }

    /**
     * real gamer field relative driving
     * @param ySpeed
     * @param xSpeed
     * @param turnSpeed
     **/

    public void FieldRelative(double ySpeed, double xSpeed, double turnSpeed) {
        double angle = (robot.gyroNavigator.getAngleGood()) + (-90);
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

        if (input.magnitude() != 0) {
            double maxMagnitude = Math.abs(frontLeftPower);
            if (Math.abs(frontRightPower) > maxMagnitude) {
                maxMagnitude = Math.abs(frontRightPower);
            }
            if (Math.abs(backLeftPower) > maxMagnitude) {
                maxMagnitude = Math.abs(backLeftPower);
            }
            if (Math.abs(backRightPower) > maxMagnitude) {
                maxMagnitude = Math.abs(backLeftPower);
            }

            frontRightPower = (frontRightPower / maxMagnitude) * input.magnitude();
            frontLeftPower = (frontLeftPower / maxMagnitude) * input.magnitude();
            backRightPower = (backRightPower / maxMagnitude) * input.magnitude();
            backLeftPower = (backLeftPower / maxMagnitude) * input.magnitude();
        } else {
            double maxMagnitude = Math.abs(frontLeftPower);
            if (Math.abs(frontRightPower) > maxMagnitude) {
                maxMagnitude = Math.abs(frontRightPower);
            }
            if (Math.abs(backLeftPower) > maxMagnitude) {
                maxMagnitude = Math.abs(backLeftPower);
            }
            if (Math.abs(backRightPower) > maxMagnitude) {
                maxMagnitude = Math.abs(backLeftPower);
            }

            frontRightPower = (frontRightPower / maxMagnitude);
            frontLeftPower = (frontLeftPower / maxMagnitude);
            backRightPower = (backRightPower / maxMagnitude);
            backLeftPower = (backLeftPower / maxMagnitude);
        }

        double speed = 1;

        if (gamepad1.left_bumper) {
            speed = .4;

        } else {

            speed = 1;

        }

        robot.driveTrain.backLeftMotor.setPower(backLeftPower * speed);
        robot.driveTrain.backRightMotor.setPower(backRightPower * speed);
        robot.driveTrain.frontLeftMotor.setPower(frontLeftPower * speed);
        robot.driveTrain.frontRightMotor.setPower(frontRightPower * speed);

    }
    public double AngleWrapDeg(double degrees) {
        while (degrees < -180 ) {
            degrees += 2.0 * 180;
        }
        while (degrees > 180 ) {
            degrees -= 2.0 * 180;
        }
        return degrees;
    }


}