package org.team4631.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.team4631.ftc.teamcode.controller.DriveController;
import org.team4631.ftc.teamcode.controller.MarkerController;
import org.team4631.ftc.teamcode.hardware.HardwareRoss;
import org.team4631.ftc.teamcode.mission.LandingManager;
import org.team4631.ftc.teamcode.mission.SamplingManager;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Ross: Autonomous Silver Sensors", group = "Ross")
public class AutoRossSilverSensors extends LinearOpMode {

    private HardwareRoss hardwareRoss;
    private LandingManager landingManager;
    private SamplingManager samplingManager;
    private DriveController driveController;
    private MarkerController markerController;
    private ElapsedTime runtime;

    @Override
    public void runOpMode() throws InterruptedException {
        hardwareRoss = new HardwareRoss(this);

        landingManager = new LandingManager(hardwareRoss);
        samplingManager = new SamplingManager(hardwareRoss);
        driveController = new DriveController(hardwareRoss);
        markerController = new MarkerController(hardwareRoss);
        runtime = new ElapsedTime();

        waitForStart();

        landingManager.landRobotByTime();

        samplingManager.sampleBySensors();

        driveController.rotate(90, 0.75, false);

        driveController.driveBySensors(0.7, 31, 5, 0, false);

        driveController.rotate(30, 0.75, false);

        driveController.driveBySensors(0.7, 55, 5, 0, false);

        markerController.releaseMarker();

        // Wait a bit for marker to settle
        runtime.reset();
        while (runtime.seconds() < 3 && hardwareRoss.getLinearOpMode().opModeIsActive()) { }

        driveController.driveBySensors(0.7, -110, 10, 0, false);
    }

}