package org.team4631.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.team4631.ftc.teamcode.controller.DriveController;
import org.team4631.ftc.teamcode.controller.MineralCollectorController;
import org.team4631.ftc.teamcode.hardware.HardwareRoss;

import java.util.List;

@Autonomous(name = "Ross: Mineral Identification Test", group = "Ross")
public class MineralIdentificationTest extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

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
    //   private static final String VUFORIA_KEY = " -- YOUR NEW VUFORIA KEY GOES HERE  --- ";
    private static final String VUFORIA_KEY = "ATHd4GH/////AAABmSlHB4jy5kAypUiIHg3IBr8rjjcAKTI0Om18f0JHO4ODL+6YVWOLfsw87JxJ2aK9XvgM+H857bxmc+BxlMMZYccMVfLpJSaH4hUTgmdGmPcuhNaXTa8QJmYIFIUikeEDCCJprchErPOFNpg9uQxgQrc2Qc5axbS22hSNeftTOSlTmo9kc+3gVH92GauyJMIMW5Ridh0pdvFiOZdkTHbRPbpSaLDTrqauS7jYDJ3to84TPfCMkXGT4BK+hDuKDatl8dhO0mFfAhTcluM1tczB0dKsq2j9PcGh/xU4SZPRz3sx/9JM3/tY/4Rcb1EL3ow2zYCzPtgVOqyOQFjN8rtViU/BAzCA9/7z3XwYsbYOiZq1";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private DriveController driveController;
    private MineralCollectorController mineralCollectorController;

    private ElapsedTime runtime;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.

        HardwareRoss hardwareRoss = new HardwareRoss(this);
        driveController = new DriveController(hardwareRoss);
        mineralCollectorController = new MineralCollectorController(hardwareRoss);

        runtime = new ElapsedTime();

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            boolean onBlockTarget = false;

            while (opModeIsActive() && !onBlockTarget) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        /* Print how many objects were detected. */
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        /* Positions of minerals (-1 = left, 0 = middle, 1 = right). */
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                /* We have found the gold mineral. */
                                goldMineralX = (int) recognition.getLeft();

                                /* Estimate the angle to the gold mineral. */
                                double degrees = -recognition.estimateAngleToObject(AngleUnit.DEGREES);

                                /* Otherwise rotate to the gold mineral. */
                                driveController.rotate((int)Math.round(degrees), 0.5, false);

                                onBlockTarget = true;
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Right");
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                            }
                        }

                        telemetry.update();
                    }
                }
            }
        }

        mineralCollectorController.setExtenderMotorOut();

        runtime.reset();
        while (runtime.seconds() < 6 && hardwareRoss.getLinearOpMode().opModeIsActive()) {
            hardwareRoss.logAndShowInTelemetry("Extending arm out...", "");
        }

        mineralCollectorController.setExtenderMotorIdle();

        /*
        runtime.reset();
        while (runtime.seconds() < 6 && hardwareRoss.getLinearOpMode().opModeIsActive()) {
            hardwareRoss.logAndShowInTelemetry("Retracting arm in...", "");
        }

        mineralCollectorController.setExtenderMotorIdle();
        */

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "LogiWebcamC930e");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

}
