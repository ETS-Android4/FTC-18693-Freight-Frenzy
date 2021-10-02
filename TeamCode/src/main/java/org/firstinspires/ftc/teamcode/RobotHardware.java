package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import static android.os.SystemClock.sleep;


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
    public DcMotorEx leftBack = null;
    public DcMotorEx rightBack = null;
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

    public BNO055IMU gyro = null;
    public BNO055IMU.Parameters parameters = null;
    public VoltageSensor voltageSensor = null;


    //public double shootTPR = 28;
    public double driveTPR = 288;
    //public double shootRPS = 16;
    public double driveRPS = 100;
    //public final double maxServoPower = 1;
    //public final double maxShootVelocity = shootTPR*shootRPS;
    public final double maxDriveVelocity = driveTPR*driveRPS;
    //public double servoPower = maxServoPower;

    //public double shootVelocity = maxShootVelocity;
    public double driveVelocity = maxDriveVelocity;
    public double lowBattery = 10.5;
    public double reallyLowBattery = 9.5;
    //public double circumferenceMM = 280;
    public double circumferenceIN = 11;
    public final double driveTickPerInch = driveTPR/circumferenceIN;
    //public final double driveTickPerMillimeter = driveTPR/circumferenceMM;
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
        public static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
        public static final String[] LABELS = {
                "Ball",
                "Cube",
                "Duck",
                "Marker"
        };

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code on the next line, between the double quotes.
         */
        public static final String VUFORIA_KEY =
         "AdImucn/////AAABmS8HHciHwUqGnUCvWpNXwjWCuQC7is1XkgwGqfbHrFJZ2aUjFR69v8HR+Jqn8Ckdsi3Y2oak9H0dwlRxirfntkWVXpSag+5fuJwvx1rd4PqIpJeiZeaJJp1apcv3crUJt6Ka7o7dqHit1VLQr4ynYG5qng0Ft1TiGIrncgnZFF5IVcvcF4DPKXjF8hLIeHzB2/gylS5pKREbj+HtQUo84tr4t5tAeBVS/Q01xJJDLF3DlTX3RXbLkaMd3QVOtO6zjCNkNG8Qj6KJRv4HHT06Q+mGVCJ1hbvM/P4V4TQVsWomxi3+f4Hf6cnWSqLOSTdLag/rIYWhjZRGuzQ5d61GQgrnFevubyQmaywN1v3RyJci";
            

        /**
         * {link #vuforia} is the variable we will use to store our instance of the Vuforia
         * localization engine.
         */
        public VuforiaLocalizer vuforia;

        /**
         * {link #tfod} is the variable we will use to store our instance of the TensorFlow Object
         * Detection engine.
         */
        public TFObjectDetector tfod;

    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Constructor */
    //public RobotHardware() {

    //}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, int wheelType, boolean enableCamera) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        if(enableCamera) {
            initVuforia();
            initTfod();
        }
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);
        }
        // Define and Initialize Motors
        leftBack = hwMap.get(DcMotorEx.class, "Motor_0");
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setPower(0);      
        leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack = hwMap.get(DcMotorEx.class, "Motor_1");
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setPower(0);
        rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); 
        double Back_Motors_F = 32767 / maxDriveVelocity, Back_Motors_P = 0.1 * Back_Motors_F, Back_Motors_I = 0.1 * Back_Motors_P;
        leftBack.setVelocityPIDFCoefficients(Back_Motors_P, Back_Motors_I, 0, Back_Motors_F);
        rightBack.setVelocityPIDFCoefficients(Back_Motors_P, Back_Motors_I, 0, Back_Motors_F);
        
        if(wheelType == 2){
        leftFront = hwMap.get(DcMotorEx.class, "Motor_2");
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setPower(0);
        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront = hwMap.get(DcMotorEx.class, "Motor_3");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setPower(0); 
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        double Front_Motors_F = 32767 / maxDriveVelocity, Front_Motors_P = 0.1 * Front_Motors_F, Front_Motors_I = 0.1 * Front_Motors_P;
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
        gyro = hwMap.get(BNO055IMU.class, "imu");
        voltageSensor = hwMap.get(VoltageSensor.class, "Control Hub");


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
    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hwMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    public void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.5f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
