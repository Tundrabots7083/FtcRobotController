package org.firstinspires.ftc.teamcode.ops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.bots.TestBot;
import org.firstinspires.ftc.teamcode.components.DriveTrain;
import org.firstinspires.ftc.teamcode.components.Intake;


@TeleOp(name="BONAPARTE", group="ops")
public class BONAPARTE extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private TestBot robot = null;
    private boolean logEnableTrace = false;
    private boolean logToTelemetry = true;





    //------------------------------InitSetup?--------------------------------------------------------\\







    @Override
    public void runOpMode() {

        robot.initAll();
        robot.driveTrain.init(DriveTrain.InitType.INIT_4WD);
        robot.intake.init();

//------------------------------Direction---------------------------------------------------------\\

        //Reverse spins motors to the right Forward spins motors to the left
        /*Intake     .setDirection (DcMotorSimple.Direction.REVERSE);
        Shooter1   .setDirection (DcMotorSimple.Direction.FORWARD);
        Shooter2   .setDirection (DcMotorSimple.Direction.FORWARD); */


        robot = new TestBot(this, logEnableTrace, logToTelemetry);
        robot.logger.logInfo("runOpMode", "===== [ Start Initializing ]");

        /* Use either robot.initAll or select only the components that need initializing below */

        robot.logger.logInfo("runOpMode", "===== [ Initialization Complete ]");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.logger.logInfo("runOpMode", "===== [ Start TeleOp ]");
        runtime.reset();




//------------------------------Wobble------------------------------------------------------------\\
//non existent just bad

        while (opModeIsActive()) {

                  //field oriented gamepad stuff
            FieldRelative(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

            //sketchy intake code idk if work
            if      (gamepad1.right_trigger > .1)
            {
                robot.loader.indexer.setPosition(.65);
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
                robot.shooter.setShooterPower(-.8);
                robot.loader.indexer.setPosition(1);
            }
            else
            {
                robot.shooter.setShooterPower(0);
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

            if     (gamepad1.y)
            {
                robot.loader.indexer.setPosition(.65);
            }
            else if(gamepad1.b)
            {
                robot.loader.indexer.setPosition(1);
            }

            if      (gamepad1.dpad_down)
            {
                //change values
                robot.shooter.ShootAngle.setPosition(.75);
            }
            else if (gamepad1.dpad_up)
            {
                //change values
                robot.shooter.ShootAngle.setPosition(1);
            }

        }
    }

    /**
     * real gamer field relative driving
     * @param ySpeed
     * @param xSpeed
     * @param turnSpeed
     **/
    public void FieldRelative(double ySpeed, double xSpeed, double turnSpeed) {
        double angle = robot.gyroNavigator.getAngleGood();
        angle  = AngleWrapDeg(angle);
        angle = -angle;

        xSpeed = Range.clip(xSpeed, -1, 1);
        ySpeed = Range.clip(ySpeed, -1, 1 );
        turnSpeed = Range.clip(turnSpeed, -1, 1 );

        org.firstinspires.ftc.teamcode.geometry.Vector2d input = new org.firstinspires.ftc.teamcode.geometry.Vector2d(xSpeed,ySpeed);

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

        telemetry.addData("angle",angle);
        telemetry.update();
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