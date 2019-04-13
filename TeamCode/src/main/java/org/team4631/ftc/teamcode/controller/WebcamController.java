package org.team4631.ftc.teamcode.controller;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.team4631.ftc.teamcode.hardware.HardwareRoss;

import java.util.List;

public class WebcamController extends AbstractCommonController {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "ATHd4GH/////AAABmSlHB4jy5kAypUiIHg3IBr8rjjcAKTI0Om18f0JHO4ODL+6YVWOLfsw87JxJ2aK9XvgM+H857bxmc+BxlMMZYccMVfLpJSaH4hUTgmdGmPcuhNaXTa8QJmYIFIUikeEDCCJprchErPOFNpg9uQxgQrc2Qc5axbS22hSNeftTOSlTmo9kc+3gVH92GauyJMIMW5Ridh0pdvFiOZdkTHbRPbpSaLDTrqauS7jYDJ3to84TPfCMkXGT4BK+hDuKDatl8dhO0mFfAhTcluM1tczB0dKsq2j9PcGh/xU4SZPRz3sx/9JM3/tY/4Rcb1EL3ow2zYCzPtgVOqyOQFjN8rtViU/BAzCA9/7z3XwYsbYOiZq1";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    public WebcamController(HardwareRoss hardwareRoss) {
        super(hardwareRoss);

        reset();
    }

    public void startTracking() {
        if (tfod != null) {
            tfod.activate();
        }
    }

    public List<Recognition> getRecognitions() {
        if (tfod != null) return tfod.getUpdatedRecognitions();

        hardwareRoss.logAndShowInTelemetry("TFOD is null!", "");
        return null;
    }

    public void endTracking() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    @Override
    public void reset() {
        hardwareRoss.logAndShowInTelemetry("Resetting webcam engine and hardware.", "");

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            hardwareRoss.logAndShowInTelemetry("TFOD initialization with the selected camera device failed.", "");
        }
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareRoss.get(WebcamName.class, "LogiWebcamC930e");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareRoss.getHardwareMap().appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareRoss.getHardwareMap().appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

}
