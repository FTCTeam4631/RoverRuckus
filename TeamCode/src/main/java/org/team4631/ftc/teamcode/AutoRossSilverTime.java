package org.team4631.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.team4631.ftc.teamcode.hardware.HardwareRoss;
import org.team4631.ftc.teamcode.mission.LandingManager;
import org.team4631.ftc.teamcode.mission.SamplingManager;

@Autonomous(name = "Ross: Autonomous Silver Time", group = "Ross")
public class AutoRossSilverTime extends LinearOpMode {

    private HardwareRoss hardwareRoss;
    private LandingManager landingManager;
    private SamplingManager samplingManager;

    @Override
    public void runOpMode() throws InterruptedException {
        hardwareRoss = new HardwareRoss(this);

        landingManager = new LandingManager(hardwareRoss);
        samplingManager = new SamplingManager(hardwareRoss);

        waitForStart();

        landingManager.landRobotByTime();

        samplingManager.sampleByTime();
    }

}