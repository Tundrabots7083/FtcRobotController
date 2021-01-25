package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


//@Disabled
public class Shooter extends BotComponent {
    private String shooter1Name;
    private String shooter2Name;
    private String shooterAngle;


    public DcMotorEx Shooter1 = null;
    public DcMotorEx Shooter2 = null;
    public Servo ShootAngle = null;



    private boolean shooterEnabled = true;


    public Shooter(){

    }

    public Shooter(Logger aLogger, OpMode aOpMode,
                   String aShooter1Name, String aShooter2Name, String aShootAngle) {
        super (aLogger, aOpMode);
        shooter1Name = aShooter1Name;
        shooter2Name = aShooter2Name;
        shooterAngle = aShootAngle;




    }

    public void init() {

        //define and initialize motors

        Shooter1 = initMotorEx(shooter1Name, DcMotor.Direction.REVERSE);
        Shooter2 = initMotorEx(shooter2Name, DcMotor.Direction.REVERSE);
        ShootAngle = initServo(shooterAngle, .75);


        if (Shooter1 != null) {
            isAvailable = true;
        }
        if (Shooter2 != null) {
            isAvailable = true;
        }
        if (ShootAngle != null) {
            isAvailable = true;
        }

        logger.logInfo("Shooter", "isAvailable: %b", isAvailable);
    }


    public void setShooterPower (double power){
        Shooter2.setPower(power);
        Shooter1.setPower(power);
    }



    public void setShooterVelocity(double velocity) {
        Shooter1.setVelocity(velocity);
        Shooter2.setPower(Shooter1.getPower());
    }


}