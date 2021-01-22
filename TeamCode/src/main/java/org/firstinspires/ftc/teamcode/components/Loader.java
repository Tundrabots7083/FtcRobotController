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
    public Servo   indexer    = null;
    private String servoName;
    private String indexerName;
    double SERVO_IN_POSITION = .83;
    double SERVO_OUT_POSITION = .5;



    /* Constructor */
    public Loader() {

    }

    public Loader(Logger aLogger, OpMode aOpMode,
                  String aLoader, String aIndexer) {
        super(aLogger, aOpMode);
        servoName = aLoader;
        indexerName = aIndexer;
    }


    public void init(){
        logger.logDebug("initservo", "orangr");
        loaderServo = initServo(servoName, SERVO_IN_POSITION);
        indexer = initServo(indexerName, .65);
        if(loaderServo != null){
            isAvailable = true;
        }
        if(indexer != null){
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