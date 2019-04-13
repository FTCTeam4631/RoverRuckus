package org.team4631.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.team4631.ftc.teamcode.controller.DriveController;
import org.team4631.ftc.teamcode.hardware.HardwareRoss;

/* This is a unit test that checks to see whether the driving methods using time work. */

@Autonomous(name = "Ross: Drive Time Test", group = "Ross")

public class DriveTimeTest extends LinearOpMode {

    private HardwareRoss hardwareRoss;

    private DriveController driveController;

    @Override
    public void runOpMode() {
        hardwareRoss = new HardwareRoss(this);

        hardwareRoss.logAndShowInTelemetry("This unit test should simply move the robot forward for five seconds.", "");

        driveController = new DriveController(hardwareRoss);

        waitForStart();

        driveController.driveByTime(0.3, 5);
    }

}
