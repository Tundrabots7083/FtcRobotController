package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.components.Logger;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.components.DriveTrain;

public class Loader extends BotComponent {

    public Servo loaderServo = null;
    private String servoName;
    double SERVO_IN_POSITION = .94;
    double SERVO_OUT_POSITION = .25;



    /* Constructor */
    public Loader() {

    }

    public Loader(Logger aLogger, OpMode aOpMode,
                  String loader) {
        super(aLogger, aOpMode);
        servoName = loader;
    }


    public void init(){
        logger.logDebug("initservo", "orangr");
        loaderServo = initServo(servoName, SERVO_IN_POSITION);
        if(loaderServo != null){
            isAvailable = true;
        }

        logger .logInfo("Loader","isAvailable: %b",isAvailable);
    }

    public void sleep(int milis){
        try {
            Thread.sleep(milis);
        } catch (Exception e){}
    }


    public void load(){
        logger.logDebug("servoMoveDown", "downorange");
        loaderServo.setPosition(SERVO_OUT_POSITION);
        sleep(1000);
        loaderServo.setPosition(SERVO_IN_POSITION);
    }







}