package org.firstinspires.ftc.teamcode.Hardware;


import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.VoltageSensor;


/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class RobotHardware {
    /* Public OpMode members. */
    public DcMotorEx leftRear = null;
    public DcMotorEx rightRear = null;
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;

    public LED light1 = null;
    public LED light2 = null;
    public LED light3 = null;
    public LED light4 = null;


    public ColorSensor color = null;
    public Rev2mDistanceSensor distanceLeft = null;
    public Rev2mDistanceSensor distanceRight = null;


    public VoltageSensor voltageSensor = null;


    public double driveTPR = 288;
    public double driveRPS = 2.1;
    public final double maxDriveVelocity = driveTPR * driveRPS;
    public double driveVelocity = maxDriveVelocity;
    public double lowBattery = 10.5;
    public double reallyLowBattery = 9.5;
    public double circumferenceMM = 280;
    public final double driveTickPerMillimeter = driveTPR / circumferenceMM;
    public double circumferenceIN = 11;
    public final double driveTickPerInch = driveTPR / circumferenceIN;
    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    /* local OpMode members. */
    HardwareMap hwMap = null;


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, int wheelType) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        leftRear = hwMap.get(DcMotorEx.class, "Motor_0");
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setPower(0);
        leftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRear = hwMap.get(DcMotorEx.class, "Motor_1");
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setPower(0);
        rightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        double Back_Motors_F = 32767 / maxDriveVelocity, Back_Motors_P = 0.1 * Back_Motors_F, Back_Motors_I = 0.01 * Back_Motors_P;
        leftRear.setVelocityPIDFCoefficients(Back_Motors_P, Back_Motors_I, 0, Back_Motors_F);
        rightRear.setVelocityPIDFCoefficients(Back_Motors_P, Back_Motors_I, 0, Back_Motors_F);

        if (wheelType == 2) {
            leftFront = hwMap.get(DcMotorEx.class, "Motor_2");
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            leftFront.setPower(0);
            leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightFront = hwMap.get(DcMotorEx.class, "Motor_3");
            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFront.setPower(0);
            rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            double Front_Motors_F = 32767 / maxDriveVelocity, Front_Motors_P = 0.1 * Front_Motors_F, Front_Motors_I = 0.01 * Front_Motors_P;
            leftFront.setVelocityPIDFCoefficients(Front_Motors_P, Front_Motors_I, 0, Front_Motors_F);
            rightFront.setVelocityPIDFCoefficients(Front_Motors_P, Front_Motors_I, 0, Front_Motors_F);
        }


        // touchBottom = hwMap.get(TouchSensor.class, "Touch_0");
        // touchTop = hwMap.get(TouchSensor.class, "Touch_1");
        // Define and initialize ALL installed lights.
        light1 = hwMap.get(LED.class, "Light_0");
        light2 = hwMap.get(LED.class, "Light_1");
        light3 = hwMap.get(LED.class, "Light_2");
        light4 = hwMap.get(LED.class, "Light_3");


        // Define and initialize ALL installed distance/light sensors.
        color = hwMap.get(ColorSensor.class, "Color_0");
        distanceLeft = hwMap.get(Rev2mDistanceSensor.class, "Distance_1");
        distanceRight = hwMap.get(Rev2mDistanceSensor.class, "Distance_2");

        // Define and initialize ALL internal sensors.
        voltageSensor = hwMap.get(VoltageSensor.class, "Control Hub");
        if (driveTickPerInch != 0 || driveTickPerMillimeter != 0) {
            light1.enable(true);
            light2.enable(true);
            light3.enable(true);
            light4.enable(true);
            sleep(50);
            light1.enable(false);
            light2.enable(false);
            light3.enable(false);
            light4.enable(false);

        }
    }
}
