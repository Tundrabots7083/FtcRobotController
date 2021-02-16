package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


//@Disabled
public class Intake extends BotComponent {
    private String rightIntakeName;
    private String leftIntakeName;


    public DcMotor Intake = null;
    public DcMotor Intake2 = null;


    private boolean rightIntakeEnabled = true;
    private boolean leftIntakeEnabled = true;

    public Intake(){

    }

    public Intake(Logger aLogger, OpMode aOpMode,
                  String aRightIntakeName, String aLeftIntakeName) {
        super(aLogger, aOpMode);
        rightIntakeName = aRightIntakeName;
        leftIntakeName = aLeftIntakeName;

    }

    public void init() {

        //define and initialize motors

        Intake = initMotor(rightIntakeName, DcMotor.Direction.REVERSE);
        Intake2 = initMotor(leftIntakeName, DcMotor.Direction.FORWARD);


        if (Intake != null) {
            isAvailable = true;
        }

        logger.logInfo("Intake", "isAvailable: %b", isAvailable);
    }


    public void setIntakePower (double power){

        Intake.setPower(power);
        Intake2.setPower(power);
    }




}