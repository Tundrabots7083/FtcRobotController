package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class GyroNavigator extends BotComponent {

    private String imuName = "imu";
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle, power = .30, correction;

    /* Constructor */
    public GyroNavigator() {

    }

    public GyroNavigator(Logger aLogger, OpMode aOpMode)
    {
        super(aLogger, aOpMode);
    }

    public GyroNavigator(Logger aLogger, OpMode aOpMode, String aImuName)
    {
        super(aLogger, aOpMode);
        imuName = aImuName;
    }

    public void init() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = opMode.hardwareMap.get(BNO055IMU.class, imuName);

        imu.initialize(parameters);

        logger.logInfo("GyroNavigator:init", "Calibrating...");

        // make sure the imu gyro is calibrated before continuing.
        while (opModeIsActive() && !imu.isGyroCalibrated())
        {
            pause(.5);
            idle();
        }

        resetAngle();

        isAvailable = true;
        logger.logInfo("GyroNavigator:init", "Calibration Status:%s",imu.getCalibrationStatus().toString());

    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. - = left, + = right.
     */
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return 0 - globalAngle;
    }


    public double getAngleCorrection(double targetAngle)
    {

        double correction, currentAngle, gain = .10;

        currentAngle = getAngle();

        correction = targetAngle - currentAngle;
        correction = correction * gain;

        return correction;
    }



}