package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


//@Disabled
public class Shooter extends BotComponent {
    private String shooter1Name;
    private String shooter2Name;



    public DcMotor Shooter1 = null;
    public DcMotor Shooter2 = null;



    private boolean shooterEnabled = true;


    public Shooter(){

    }

    public Shooter(Logger aLogger, OpMode aOpMode,
                   String aShooter1Name, String aShooter2Name) {
        super (aLogger, aOpMode);
        shooter1Name = aShooter1Name;
        shooter2Name = aShooter2Name;




    }

    public void init() {

        //define and initialize motors

        Shooter1 = initMotor(shooter1Name, DcMotor.Direction.REVERSE);
        Shooter2 = initMotor(shooter2Name, DcMotor.Direction.REVERSE);


        if (Shooter1 != null) {
            isAvailable = true;
        }
        if (Shooter2 != null) {
            isAvailable = true;
        }

        logger.logInfo("Shooter", "isAvailable: %b", isAvailable);
    }


    public void setShooterPower (double power){
        Shooter2.setPower(-power);
        Shooter1.setPower(-power);
    }




}