package org.firstinspires.ftc.teamcode.bots;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class Bot {

    public OpMode opMode = null;

    //Constructor//
    public Bot() {

    }
    public Bot(OpMode aOpMode) {
        opMode = aOpMode;
    }
}