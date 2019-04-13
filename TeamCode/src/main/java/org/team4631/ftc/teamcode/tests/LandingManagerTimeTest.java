package org.team4631.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.team4631.ftc.teamcode.controller.DriveController;
import org.team4631.ftc.teamcode.hardware.HardwareRoss;
import org.team4631.ftc.teamcode.mission.LandingManager;

/* This is a unit test that checks to see whether the landing manager using time works. */

@Autonomous(name = "Ross: Landing Manager Time Test", group = "Ross")

public class LandingManagerTimeTest extends LinearOpMode {

    private HardwareRoss hardwareRoss;

    private LandingManager landingManager;

    @Override
    public void runOpMode() {
        hardwareRoss = new HardwareRoss(this);

        hardwareRoss.logAndShowInTelemetry("This unit test should simply do the landing sequence using time.", "");

        landingManager = new LandingManager(hardwareRoss);

        waitForStart();

        landingManager.landRobotByTime();
    }

}
