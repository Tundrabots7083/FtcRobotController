package org.firstinspires.ftc.teamcode.ops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.bots.TestBot;
import org.firstinspires.ftc.teamcode.components.DriveTrain;


@TeleOp(name="NapoleonTeleOp", group="ops")
public class NapoleonTeleOp extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private TestBot robot = null;
    private boolean logEnableTrace = false;
    private boolean logToTelemetry = true;


    @Override
    public void runOpMode() {

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


        while (opModeIsActive()) {

            /********** Put Your Code Here **********/
/*
                if (gamepad1.x) {
                  robot.driveTrain.frontRightMotor.setPower(1);
                   robot.driveTrain.backLeftMotor.setPower(1);
                    robot.driveTrain.backRightMotor.setPower(1);
                    robot.driveTrain.frontLeftMotor.setPower(1);
                }
                double leftX = gamepad1.left_stick_x;
                double leftY = gamepad1.left_stick_y;
                double rightX = gamepad1.right_stick_x;
                double rightY = gamepad1.right_stick_y;

                robot.driveTrain.updateMotorsMechanumDrive(leftX, leftY, rightX, rightY);

                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Left", "X (%.2f), Y (%.2f)", leftX, leftY);
                telemetry.addData("Right", "X (%.2f), Y (%.2f)", rightX, rightY);

            //normal mecanum code
*/
            //intake
            if (robot.intake.isAvailable) {
                if (gamepad1.right_trigger > 0) {
                    robot.intake.setIntakePower(1);
                }
                if (gamepad1.right_trigger <= 0) {
                    robot.intake.setIntakePower(0);
                }
            }

            //intake reverse
            if (gamepad1.left_bumper) {
                robot.intake.setIntakePower(-1);
            }

            //shooter
            if (robot.shooter.isAvailable) {
                double triggerPressure = gamepad1.right_trigger;

                robot.shooter.setShooterPower(triggerPressure);
            }

            //loader
            if (gamepad1.right_bumper){
                robot.loader.load();
            }

            if (gamepad1.a) {
                robot.shooter.setShooterPower(0.25);
            }
            if (gamepad1.b) {
                robot.shooter.setShooterPower(.8);
            }


        }
    }
}