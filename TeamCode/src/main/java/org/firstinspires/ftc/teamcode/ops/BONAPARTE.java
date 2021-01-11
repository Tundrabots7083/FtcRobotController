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


@TeleOp(name="BONAPARTE", group="ops")
public class BONAPARTE extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private TestBot robot = null;
    private boolean logEnableTrace = false;
    private boolean logToTelemetry = true;





    //------------------------------InitSetup?--------------------------------------------------------\\


    public DcMotor Intake     = null;
    public DcMotor Shooter1   = null;
    public DcMotor Shooter2   = null;
    public Servo   ShootAngle = null;
    public Servo   loader     = null;
    public Servo   indexer    = null;
    public Servo   wobbleArm    = null;
    public Servo wobbleClaw     = null;


    @Override
    public void runOpMode() {


//------------------------------PhoneHardWareMap--------------------------------------------------\\


        Intake      = hardwareMap.dcMotor.get ( "rightIntake ");
        Shooter1    = hardwareMap.dcMotor.get ( "shooterOne        ");
        Shooter2    = hardwareMap.dcMotor.get ( "shooterTwo        ");
        loader      = hardwareMap.servo.get   ( "loader            ");
        ShootAngle  = hardwareMap.servo.get   ( "shooterAngle      ");
        indexer     = hardwareMap.servo.get   ( "indexer           ");
        wobbleArm   = hardwareMap.servo.get   ( "wobbleArm           ");
        wobbleClaw  = hardwareMap.servo.get   ( "wobbleClaw          ");


//------------------------------Direction---------------------------------------------------------\\

        //Reverse spins motors to the right Forward spins motors to the left
        Intake     .setDirection (DcMotorSimple.Direction.REVERSE);
        Shooter1   .setDirection (DcMotorSimple.Direction.FORWARD);
        Shooter2   .setDirection (DcMotorSimple.Direction.FORWARD);


        robot = new TestBot(this, logEnableTrace, logToTelemetry);
        robot.logger.logInfo("runOpMode", "===== [ Start Initializing ]");

        /* Use either robot.initAll or select only the components that need initializing below */
        robot.initAll();
        robot.driveTrain.init(DriveTrain.InitType.INIT_4WD);
        robot.intake.init();

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
                indexer.setPosition(.65);
                Intake.setPower(1);
            }
            else if (gamepad1.left_trigger  > .1)
            {
                Intake.setPower(-1);
            }
            else
            {
                Intake.setPower(0);
            }

            if (gamepad1.left_bumper)
            {
                Shooter1.setPower(-.8);
                Shooter2.setPower(-.8);
                indexer.setPosition(1);
            }
            else
            {
                Shooter1.setPower(0);
                Shooter2.setPower(0);
            }

            if (gamepad1.right_bumper)
            {
                //change values
                loader.setPosition(.5);
            }
            else
            {
                //change value
                loader.setPosition(.83);
            }

            if     (gamepad1.y)
            {
                indexer.setPosition(.65);
            }
            else if(gamepad1.b)
            {
                indexer.setPosition(1);
            }

            if      (gamepad1.dpad_down)
            {
                //change values
                ShootAngle.setPosition(.75);
            }
            else if (gamepad1.dpad_up)
            {
                //change values
                ShootAngle.setPosition(1);
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