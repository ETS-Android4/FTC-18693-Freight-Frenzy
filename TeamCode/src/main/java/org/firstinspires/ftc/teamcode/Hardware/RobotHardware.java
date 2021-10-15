package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
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

    /*public CRServo rampBottom = null;
    public CRServo rampMiddle = null;
    public CRServo rampTop = null;
    public CRServo clawArm = null;
    public CRServo clawHand = null;
*/
   /* public TouchSensor touchBottom = null;
    public TouchSensor touchTop = null;
    */
    
   /* public LED redLight = null;
    public LED blueLight = null;
    public LED greenLight = null;
    public LED spareLight1 = null;
    public LED spareLight2 = null;
    public LED spareLight3 = null;
    public LED spareLight4 = null;
    public LED spareLight5 = null;
*/

    public ColorSensor color = null;
    public Rev2mDistanceSensor distanceLeft = null;
    public Rev2mDistanceSensor distanceRight = null;


    public VoltageSensor voltageSensor = null;


    //public double shootTPR = 28;
    public double driveTPR = 288;
    //public double shootRPS = 16;
    public double driveRPS = 2.1;
    //public final double maxServoPower = 1;
    //public final double maxShootVelocity = shootTPR*shootRPS;
    public final double maxDriveVelocity = driveTPR * driveRPS;
    //public double servoPower = maxServoPower;

    //public double shootVelocity = maxShootVelocity;
    public double driveVelocity = maxDriveVelocity;
    public double lowBattery = 10.5;
    public double reallyLowBattery = 9.5;
    public double circumferenceMM = 280;
    public final double driveTickPerMillimeter = driveTPR / circumferenceMM;
    public double circumferenceIN = 11;
    public final double driveTickPerInch = driveTPR / circumferenceIN;
    //public final double armRatio = 6;
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

    /* Constructor */
    //public RobotHardware() {

    //}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, int wheelType) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        leftRear = hwMap.get(DcMotorEx.class, "Motor_0");
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setPower(0);
        leftRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightRear = hwMap.get(DcMotorEx.class, "Motor_1");
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setPower(0);
        rightRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        double Back_Motors_F = 32767 / maxDriveVelocity, Back_Motors_P = 0.1 * Back_Motors_F, Back_Motors_I = 0.01 * Back_Motors_P;
        leftRear.setVelocityPIDFCoefficients(Back_Motors_P, Back_Motors_I, 0, Back_Motors_F);
        rightRear.setVelocityPIDFCoefficients(Back_Motors_P, Back_Motors_I, 0, Back_Motors_F);

        if (wheelType == 2) {
            leftFront = hwMap.get(DcMotorEx.class, "Motor_2");
            leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
            leftFront.setPower(0);
            leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightFront = hwMap.get(DcMotorEx.class, "Motor_3");
            rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
            rightFront.setPower(0);
            rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            double Front_Motors_F = 32767 / maxDriveVelocity, Front_Motors_P = 0.1 * Front_Motors_F, Front_Motors_I = 0.01 * Front_Motors_P;
            leftFront.setVelocityPIDFCoefficients(Front_Motors_P, Front_Motors_I, 0, Front_Motors_F);
            rightFront.setVelocityPIDFCoefficients(Front_Motors_P, Front_Motors_I, 0, Front_Motors_F);
        }

        // Define and initialize ALL installed servos.
       /* rampBottom = hwMap.get(CRServo.class, "Servo_0");
        rampMiddle = hwMap.get(CRServo.class, "Servo_1");
        rampTop = hwMap.get(CRServo.class, "Servo_2");
        clawArm = hwMap.get(CRServo.class, "Servo_3");
        clawHand = hwMap.get(CRServo.class, "Servo_4");
        */
        // Define and initialize ALL installed touch sensors.
       /* touchBottom = hwMap.get(TouchSensor.class, "Touch_0");
        touchTop = hwMap.get(TouchSensor.class, "Touch_1");
        */
        // Define and initialize ALL installed lights.
       /* redLight = hwMap.get(LED.class, "Light_0");
        blueLight = hwMap.get(LED.class, "Light_1");
        greenLight = hwMap.get(LED.class, "Light_2");
        spareLight1 = hwMap.get(LED.class, "Light_3");
        spareLight2 = hwMap.get(LED.class, "Light_4");
        spareLight3 = hwMap.get(LED.class, "Light_5");
        spareLight4 = hwMap.get(LED.class, "Light_6");
        spareLight5 = hwMap.get(LED.class, "Light_7");

        */
        // Define and initialize ALL installed distance/light sensors.
        color = hwMap.get(ColorSensor.class, "Color_0");
        distanceLeft = hwMap.get(Rev2mDistanceSensor.class, "Distance_1");
        distanceRight = hwMap.get(Rev2mDistanceSensor.class, "Distance_2");

        // Define and initialize ALL internal sensors.
        voltageSensor = hwMap.get(VoltageSensor.class, "Control Hub");

    }
}
