package org.team4631.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.team4631.ftc.teamcode.hardware.HardwareRoss;
import org.team4631.ftc.teamcode.mission.PathPlanningManager;

/* This is a unit test that tests the path planning manager by moving the robot through a smooth curve. */

@Autonomous(name = "Ross: Path Planning Test", group = "Ross")
public class PathPlanningTest extends LinearOpMode {

    private HardwareRoss hardwareRoss;

    private PathPlanningManager pathPlanningManager;

    @Override
    public void runOpMode() {
        hardwareRoss = new HardwareRoss(this);

        hardwareRoss.logAndShowInTelemetry("This unit test should move the robot in smooth curve.", "");

        pathPlanningManager = new PathPlanningManager(hardwareRoss);

        waitForStart();

        pathPlanningManager.followPath(58, 90, 11, 133, 15, 0.1, 18);
    }

}
