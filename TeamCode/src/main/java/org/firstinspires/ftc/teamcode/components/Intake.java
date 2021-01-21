package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


//@Disabled
public class Intake extends BotComponent {
    private String rightIntakeName;


    public DcMotor Intake = null;



    private boolean rightIntakeEnabled = true;


    public Intake(){

    }

    public Intake(Logger aLogger, OpMode aOpMode,
                  String aRightIntakeName) {
        super (aLogger, aOpMode);
        rightIntakeName = aRightIntakeName;




    }

    public void init() {

        //define and initialize motors

        Intake = initMotor(rightIntakeName, DcMotor.Direction.REVERSE);


        if (Intake != null) {
            isAvailable = true;
        }

        logger.logInfo("Intake", "isAvailable: %b", isAvailable);
    }


    public void setIntakePower (double power){

        Intake.setPower(power);
    }




}