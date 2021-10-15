package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class CameraHardware {
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
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    public VuforiaLocalizer vuforia;
    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    public TFObjectDetector tfod;
    public List<String> objects;
    public List<Float> position;
    public Position position2 = null;
    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap, float confidence, double magnification) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        initVuforia();
        initTfod(confidence);
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(magnification, 16.0 / 9.0);

        }
    }

    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hwMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    public void initTfod(float confidence) {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = confidence;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    public List<String> getObjects() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                //telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                objects.clear();
                for (Recognition recognition : updatedRecognitions) {
                    objects.add(recognition.getLabel());
                    //telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    //telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                    //recognition.getLeft(), recognition.getTop());
                    // telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                    //recognition.getRight(), recognition.getBottom());
                    i++;
                }
                return objects;
            } else {
                return objects;
            }
        }
        return null;
    }

    public Position getPosition(String object) {
        position.clear();
        position2 = null;
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                //telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(object)) {
                        position.add(recognition.getRight());
                        position.add(recognition.getBottom());
                        position2.x = recognition.getRight();
                        position2.y = recognition.getBottom();
                        //position2.z = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                        position2.unit = DistanceUnit.INCH;
                        //        position2.acquisitionTime = getRuntime();
                        return position2;
                    }
                    i++;
                }
            }
        }
        return null;
    }
}