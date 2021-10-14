package org.firstinspires.ftc.teamcode.Hardware;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import static android.os.SystemClock.sleep;

public class GyroHardware {
    HardwareMap hwMap = null;
    public BNO055IMU gyro = null;
    public BNO055IMU.Parameters parameters = null;
    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;
        gyro = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        parameters.temperatureUnit      = BNO055IMU.TempUnit.FARENHEIT;
        gyro.initialize(parameters);

        while (!gyro.isGyroCalibrated())
        {
            sleep(50);
        }

    }
    public Orientation getOrientation(){
        if(gyro.isGyroCalibrated()){
            Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            return angles;
        }
        return null;
    }
}
