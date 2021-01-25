
package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class BotComponent {

    public OpMode opMode = null;
    public Logger logger = null;
    public Boolean isAvailable = null;

    private boolean checkedOpMode = false;
    private LinearOpMode linearOpMode = null;

    /* Constructor */
    public BotComponent() {

    }

    public BotComponent(OpMode aOpMode) {
        opMode = aOpMode;
    }

    public BotComponent(Logger aLogger, OpMode aOpMode) {
        logger = aLogger;
        opMode = aOpMode;
        isAvailable = false;
    }

    private LinearOpMode getLinearOpMode() throws ClassCastException {
        LinearOpMode op = (LinearOpMode) opMode;
        return op;
    }

    public boolean opModeIsActive() {
        if (!checkedOpMode) {
            try {
                linearOpMode = getLinearOpMode();
            } catch (ClassCastException err) {
                linearOpMode = null;
            }
            checkedOpMode = true;
        }

        if (linearOpMode == null) {
            return true;
            // return !(Thread.currentThread().isInterrupted());
        } else {
            return linearOpMode.opModeIsActive();

        }
    }


    public DcMotor initMotor(String motorName) {
        return(initMotor(motorName, DcMotor.Direction.FORWARD));
    }


    public DcMotor initMotor(String motorName, DcMotorSimple.Direction direction) {
        return initMotor(motorName, direction, false);
    }

    public DcMotorEx initMotorEx(String motorName, DcMotorEx.Direction direction) {
        return initMotorEx(motorName,direction,false);
    }

    public DcMotor initMotor(String motorName, DcMotorSimple.Direction direction, boolean resetEncoder) {
        try {
            HardwareMap ahwMap = opMode.hardwareMap;
            DcMotor motor = ahwMap.get(DcMotor.class, motorName);

            motor.setDirection(direction);
            motor.setPower(0);
            if (resetEncoder) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            return (motor);

        } catch (NullPointerException | IllegalArgumentException err) {
            if (opMode.telemetry != null) {
                logger.logErr("initMotor","Error: %s", err.getMessage());
            }
            return null;
        }
    }
    public DcMotorEx initMotorEx(String motorName, DcMotorEx.Direction direction, boolean resetEncoder) {
        try {
            HardwareMap ahwMap = opMode.hardwareMap;
            DcMotorEx motor = ahwMap.get(DcMotorEx.class, motorName);

            motor.setDirection(direction);
            motor.setPower(0);
            if (resetEncoder) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            return (motor);

        } catch (NullPointerException | IllegalArgumentException err) {
            if (opMode.telemetry != null) {
                logger.logErr("initMotor","Error: %s", err.getMessage());
            }
            return null;
        }
    }

    public Servo initServo(String servoName, double position) {
        try {
            HardwareMap ahwMap = opMode.hardwareMap;
            Servo servo = ahwMap.get(Servo.class, servoName);
            servo.setPosition(position);

            return (servo);

        } catch (NullPointerException | IllegalArgumentException err) {
            if (opMode.telemetry != null) {
                logger.logErr("initServo","Error: %s", err.getMessage());
            }
            return null;
        }
    }

    public TouchSensor initTouchSensor(String sensorName) {
        try {
            HardwareMap ahwMap = opMode.hardwareMap;
            TouchSensor touchSensor = ahwMap.get(TouchSensor.class, sensorName);
            return (touchSensor);
        } catch (NullPointerException | IllegalArgumentException err) {
            if (opMode.telemetry != null) {
                logger.logErr("initTouchSensor","Error: %s", err.getMessage());
            }
            return null;
        }
    }

    public void pause(double seconds) {
        long milliseconds = (long)seconds * 1000;

        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

    }

    public final void idle() {
        // Otherwise, yield back our thread scheduling quantum and give other threads at
        // our priority level a chance to run
        Thread.yield();
    }
}