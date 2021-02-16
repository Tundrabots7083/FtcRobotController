package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.bots.TestBot;
import org.firstinspires.ftc.teamcode.components.DriveTrain;

@TeleOp
public class WobbleBad extends LinearOpMode {
    //shooter PID values
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.0016, 0, 0.0000016);

    //shooter feedforward values
    public static double kV = 0.000435 / TuningController.rpmToTicksPerSecond(TuningController.MOTOR_MAX_RPM);
    public static double kA = 0.00015;
    public static double kStatic = 0;

    // Timer for calculating desired acceleration - Necessary for kA to have an affect
    private final ElapsedTime veloTimer = new ElapsedTime();
    private double lastTargetVelo = 0.0;

    //velocity controller
    private final VelocityPIDFController veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);
    // access motors from anywhere in the class
    private DcMotorEx myMotor1;
    private DcMotorEx myMotor2;
    //telemetry
    private ElapsedTime runtime = new ElapsedTime();
    private TestBot robot = null;
    private boolean logEnableTrace = false;
    private boolean logToTelemetry = true;

    //servo init
    public Servo wobbleArm;
    public Servo wobbleClaw;


    // last set powers / positions for lynx optimization, set to big dumb number so things dont break
    private double lastIntakePower = 10000000;
    private double lastShooterPower = 10000000;
    private double lastIndexerPosition = 10000000;
    private double lastLoaderPosition = 10000000;
    private double lastShootAnglePosition = 1000000;
    private double lastFlPower = 1000000;
    private double lastFrPower = 1000000;
    private double lastBlPower = 1000000;
    private double lastBrPower = 1000000;

    @Override
    public void runOpMode() throws InterruptedException {
        //shooter motor setup
        myMotor1 = hardwareMap.get(DcMotorEx.class, "shooterOne");
        myMotor2 = hardwareMap.get(DcMotorEx.class, "shooterTwo");

        myMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Turns on bulk reading
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //tele init
        robot = new TestBot(this, logEnableTrace, logToTelemetry);
        robot.logger.logInfo("runOpMode", "===== [ Start Initializing ]");

        /* Use either robot.initAll or select only the components that need initializing below */

        robot.logger.logInfo("runOpMode", "===== [ Initialization Complete ]");
        telemetry.update();

        wobbleClaw = hardwareMap.get(Servo.class, "wobbleClaw");
        wobbleArm = hardwareMap.get(Servo.class, "wobbleArm");


        waitForStart();

        //hardware map init
        robot.shooter.init();
        robot.loader.init();
        robot.intake.init();
        robot.driveTrain.init(DriveTrain.InitType.INIT_4WD);
        robot.gyroNavigator.init();
        robot.logger.logInfo("runOpMode", "===== [ Start TeleOp ]");
        runtime.reset();

        //init servo positions
        wobbleArm.setPosition(.2);
        robot.shooter.ShootAngle.setPosition(.84);


        if (isStopRequested()) return;

        // Start the veloTimer
        veloTimer.reset();
        ElapsedTime loopTimer = new ElapsedTime();
        while (!isStopRequested()) {
            loopTimer.reset();
            ///// Run the velocity controller ////
            // Target velocity in ticks per second

            double SHOOTER_RPM = 4525;
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

            //field oriented gamepad stuff
            FieldRelative(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

            //intake
            if (gamepad1.right_trigger > .1) {
                setIndexerPositionLynxOptmized(.71);
                setIntakePowerLynxOptimized(1);
            } else if (gamepad1.left_trigger > .1) {
                setIntakePowerLynxOptimized(-1);
            } else {
                setIntakePowerLynxOptimized(0);
            }

            if (gamepad1.left_bumper) {
                setShooterPowerLynxOptimized(power);
                setIndexerPositionLynxOptmized(1);
            } else {
                setShooterPowerLynxOptimized(0);
            }

            if (gamepad1.right_bumper) {
                //in
                setLoaderPositionLynxOptmized(.67);
            } else {
                //out
                setLoaderPositionLynxOptmized(.83);
            }

            if (gamepad1.dpad_down) {
                //powershots
                setShootAnglePositionLynxOptimized(.81);
            } else if (gamepad1.dpad_up) {
                //highgoal
                setShootAnglePositionLynxOptimized(.84);
            }

            if (gamepad1.y) {
                //wobble arm down
                wobbleArm.setPosition(.4);
                sleep(200);
                wobbleClaw.setPosition(.45);
            } else if (gamepad1.b) {
                //wobble arm up
                wobbleClaw.setPosition(0.15);
                sleep(200);
                wobbleArm.setPosition(0);
            }


            telemetry.addData("LOOP TIME MS: ",loopTimer.milliseconds());
            telemetry.update();
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

        //slow button
        if (gamepad1.left_bumper) {
            speed = .4;

        } else {

            speed = 1;

        }

        setDrivePowerLynxOptmized(frontLeftPower * speed,frontRightPower * speed,
                backLeftPower * speed,backRightPower * speed);

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


    /**
     * set power of the intake while optimizing lynx writes
     * @param power
     */
    public void setIntakePowerLynxOptimized(double power) {
        if (power != lastIntakePower) {
            robot.intake.setIntakePower(power);
        }
        lastIntakePower = power;
    }

    public void setShooterPowerLynxOptimized(double power) {
        if (power != lastShooterPower) {
            myMotor1.setPower(power);
            myMotor2.setPower(power);
        }
        lastShooterPower = power;
    }

    public void setIndexerPositionLynxOptmized(double position) {
        if (position != lastIndexerPosition) {
            robot.loader.indexer.setPosition(position);
        }
        lastIndexerPosition = position;

    }

    public void setLoaderPositionLynxOptmized(double position) {
        if (position != lastLoaderPosition) {
            robot.loader.loaderServo.setPosition(position);
        }
        lastLoaderPosition = position;
    }

    public void setShootAnglePositionLynxOptimized(double position) {
        if (position != lastShootAnglePosition) {
            robot.shooter.ShootAngle.setPosition(position);
        }
        lastShootAnglePosition = position;

    }

    /**
     * set drive train power lynx optmizied
     * @param fl front left
     * @param fr front right
     * @param bl back left
     * @param br back right
     */
    public void setDrivePowerLynxOptmized(double fl, double fr, double bl, double br) {
        if (lastFlPower != fl) {
            robot.driveTrain.backLeftMotor.setPower(fl);
        }

        if (lastFrPower != fr) {
            robot.driveTrain.backRightMotor.setPower(fr);
        }

        if (lastBlPower != bl) {
            robot.driveTrain.backLeftMotor.setPower(bl);
        }

        if (lastBrPower != br) {
            robot.driveTrain.backRightMotor.setPower(br);
        }

        lastFrPower = fr;
        lastFlPower = fl;
        lastBlPower = bl;
        lastBrPower = br;
    }


}