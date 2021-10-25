package org.firstinspires.ftc.teamcode.Hardware;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

public class CameraHardware {
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
    public boolean initialized = false;
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    private static final String VUFORIA_KEY = "AdImucn/////AAABmS8HHciHwUqGnUCvWpNXwjWCuQC7is1XkgwGqfbHrFJZ2aUjFR69v8HR+Jqn8Ckdsi3Y2oak9H0dwlRxirfntkWVXpSag+5fuJwvx1rd4PqIpJeiZeaJJp1apcv3crUJt6Ka7o7dqHit1VLQr4ynYG5qng0Ft1TiGIrncgnZFF5IVcvcF4DPKXjF8hLIeHzB2/gylS5pKREbj+HtQUo84tr4t5tAeBVS/Q01xJJDLF3DlTX3RXbLkaMd3QVOtO6zjCNkNG8Qj6KJRv4HHT06Q+mGVCJ1hbvM/P4V4TQVsWomxi3+f4Hf6cnWSqLOSTdLag/rIYWhjZRGuzQ5d61GQgrnFevubyQmaywN1v3RyJci";

    /**
     * {link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    public TFObjectDetector tfod;
    public int ObjectsDetected;
    public String ObjectName;
    public List<Integer> CamLeft = new ArrayList<Integer>();
    public List<Integer> CamTop = new ArrayList<Integer>();
    public List<Integer> CamRight = new ArrayList<Integer>();
    public List<Integer> CamBottom = new ArrayList<Integer>();
    public int ObjectNum;
    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap, double magnification) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        initVuforia();
        initTfod();
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
        initialized = true;
    }

    private void initVuforia() {
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
    private void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    @SuppressLint("DefaultLocale")
    public void findObject() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                ObjectsDetected = updatedRecognitions.size();
                // step through the list of recognitions and display boundary info.
                int i = 0;
                ObjectNum = 0;
                for (Recognition recognition : updatedRecognitions) {
                    ObjectName = recognition.getLabel();
                    CamLeft.add(Math.round(recognition.getLeft()));
                    CamTop.add(Math.round(recognition.getTop()));
                    CamRight.add(Math.round(recognition.getLeft()));
                    CamBottom.add(Math.round(recognition.getTop()));
                    i++;
                    ObjectNum = i;
                }
            }
        }
    }
}