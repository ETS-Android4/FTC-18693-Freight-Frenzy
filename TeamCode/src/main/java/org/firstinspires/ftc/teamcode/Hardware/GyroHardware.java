package org.firstinspires.ftc.teamcode.Hardware;

import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Temperature;

public class GyroHardware {
    public BNO055IMU gyro = null;
    public BNO055IMU.Parameters parameters = null;
    public Temperature temp;
    public Orientation angles;
    public Orientation angles2;
    public boolean initialized = false;
    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        gyro = hwMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.temperatureUnit = BNO055IMU.TempUnit.FARENHEIT;
        //parameters.gyroPowerMode        = BNO055IMU.GyroPowerMode.ADVANCED;
        gyro.initialize(parameters);

        while (!gyro.isGyroCalibrated()) {
            sleep(50);
        }
        initialized = true;
    }

    public Orientation getOrientation() {
        if (gyro.isGyroCalibrated()) {
            angles = gyro.getAngularOrientation(AxesReference.INTRINSIC.reverse(), AxesOrder.XYZ, AngleUnit.DEGREES);
            return angles;
        }
        return null;
    }

    public Orientation getOrientation2() {
        if (gyro.isGyroCalibrated()) {
            angles2 = gyro.getAngularOrientation(AxesReference.EXTRINSIC.reverse(), AxesOrder.XYZ, AngleUnit.DEGREES);
            return angles2;
        }
        return null;
    }

    public double getTemp() {
        temp = gyro.getTemperature();
        return temp.temperature;
    }
}
