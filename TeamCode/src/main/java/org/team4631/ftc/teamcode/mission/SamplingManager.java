package org.team4631.ftc.teamcode.mission;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.team4631.ftc.teamcode.controller.DriveController;
import org.team4631.ftc.teamcode.controller.MineralCollectorController;
import org.team4631.ftc.teamcode.controller.WebcamController;
import org.team4631.ftc.teamcode.hardware.HardwareRoss;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;

public class SamplingManager {

    private static final int SAMPLES_PER_CHECK = 75;
    private static final double SAMPLE_TIMEOUT_PER_CHECK = 1.5;

    private HardwareRoss hardwareRoss;
    private DriveController driveController;
    private WebcamController webcamController;
    private MineralCollectorController mineralCollectorController;

    private ElapsedTime runtime;

    public SamplingManager(HardwareRoss hardwareRoss) {
        this.hardwareRoss = hardwareRoss;
        this.driveController = new DriveController(hardwareRoss);
        this.webcamController = new WebcamController(hardwareRoss);
        this.mineralCollectorController = new MineralCollectorController(hardwareRoss);
        this.runtime = new ElapsedTime();
    }

    // Returns array of doubles where first element determines if gold was recognized and second element determines the angle turned to gold
    private double[] turnToGoldRecognition() {
        double ret[] = new double[2];

        ret[0] = 0;
        ret[1] = 0;

        // Option 1

        /*
        for (int i = 0; i < SAMPLES_PER_CHECK; i++) {
            List<Recognition> recognitions = webcamController.getRecognitions();

            if (recognitions == null) {
                hardwareRoss.logAndShowInTelemetry("Recognitions is null!", "");
                continue;
            }

            for (Recognition r : recognitions) {
                if (r.getLabel().equals(LABEL_GOLD_MINERAL)) {
                    ret[0] = 1;
                    // Estimate the angle to the gold mineral.
                    ret[1] = -r.estimateAngleToObject(AngleUnit.DEGREES);

                    // Rotate to the gold mineral.
                    driveController.rotate((int) Math.round(ret[1]), 0.5, false);

                    return ret;
                }
            }
        }
        */

        runtime.reset();

        while (runtime.seconds() < SAMPLE_TIMEOUT_PER_CHECK && hardwareRoss.getLinearOpMode().opModeIsActive()) {
            List<Recognition> recognitions = webcamController.getRecognitions();

            if (recognitions == null) {
                hardwareRoss.logAndShowInTelemetry("Recognitions is null!", "");
                continue;
            }

            for (Recognition r : recognitions) {
                if (r.getLabel().equals(LABEL_GOLD_MINERAL)) {
                    ret[0] = 1;
                    // Estimate the angle to the gold mineral.
                    ret[1] = -r.estimateAngleToObject(AngleUnit.DEGREES);

                    hardwareRoss.logAndShowInTelemetry("Found gold mineral @ ", ret[1]);

                    // Rotate to the gold mineral.
                    driveController.rotate((int)Math.round(ret[1]), 0.5, false);

                    return ret;
                }
            }
        }

        return ret;
    }

    public void sample() {
        webcamController.startTracking();

        double globalDeltaAngle = 0.0;

        double[] recognitionData = turnToGoldRecognition();

        // Found gold mineral in center
        if (recognitionData[0] == 1) {
            globalDeltaAngle += recognitionData[1];
        } else {
            driveController.rotate(20, 0.5, false);

            globalDeltaAngle += 20;

            recognitionData = turnToGoldRecognition();

            if (recognitionData[0] == 1) {
                globalDeltaAngle += recognitionData[1];
            } else {
                driveController.rotate(-40, 0.5, false);

                globalDeltaAngle -= 40;

                recognitionData = turnToGoldRecognition();

                if (recognitionData[0] == 1) {
                    globalDeltaAngle += recognitionData[1];
                } else {
                    hardwareRoss.logAndShowInTelemetry("No gold mineral found!", "");
                    return;
                }
            }
        }

        mineralCollectorController.setStandServoRelease();

        mineralCollectorController.setExtenderMotorOut();

        runtime.reset();
        while (runtime.seconds() < 6 && hardwareRoss.getLinearOpMode().opModeIsActive()) {
            hardwareRoss.logAndShowInTelemetry("Extending arm out...", "");
        }

        mineralCollectorController.setExtenderMotorIdle();

        mineralCollectorController.setTiltServoUpright();

        mineralCollectorController.setExtenderMotorIn();

        runtime.reset();
        while (runtime.seconds() < 6 && hardwareRoss.getLinearOpMode().opModeIsActive()) {
            hardwareRoss.logAndShowInTelemetry("Retracting arm in...", "");
        }

        mineralCollectorController.setExtenderMotorIdle();

        // Come back to home position (where the robot was facing when it landed)
        driveController.rotate((int)(-Math.round(globalDeltaAngle)), 0.5, false);

        webcamController.endTracking();
    }

    public void sampleByTime() {
        driveController.driveByTime(0.5, 0.3);

        sample();
    }

    public void sampleBySensors() {
        driveController.driveBySensors(0.5, 3.0, 5.0, 0.0, false);

        sample();
    }

}
