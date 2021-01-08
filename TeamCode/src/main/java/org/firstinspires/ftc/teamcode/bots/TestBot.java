package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.components.*;


public class TestBot extends Bot {

    //Bot components//

    public Logger logger = null;
    //public WebCamera webCamera = null;//
    public DriveTrain driveTrain = null;
    public GyroNavigator gyroNavigator = null;
    public Intake intake = null;
    public Loader loader = null;
    public Shooter shooter = null;

    /* Constructor */
    public TestBot() {

    }

    public TestBot(OpMode aOpMode) {
        this(aOpMode, false, false);
    }

    public TestBot(OpMode aOpMode, boolean enableTrace, boolean enableTelemetry) {

        logger = new Logger("TestBot", aOpMode, enableTrace, enableTelemetry);
        gyroNavigator = new GyroNavigator(logger, aOpMode);
        driveTrain = new DriveTrain(logger, aOpMode, "frontLeftMotor", "frontRightMotor",
                "backLeftMotor", "backRightMotor",
                gyroNavigator);
        intake = new Intake(logger, aOpMode, "rightIntake");
        loader = new Loader(logger, aOpMode, "loader");
        shooter = new Shooter(logger, aOpMode, "shooterOne", "shooterTwo");


    }

    public void initAll() {
        gyroNavigator.init();
        driveTrain.init(DriveTrain.InitType.INIT_4WD);
        intake.init();
        loader.init();
        shooter.init();
    }
}