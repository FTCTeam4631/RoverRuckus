package org.team4631.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.team4631.ftc.teamcode.controller.DriveController;
import org.team4631.ftc.teamcode.hardware.HardwareRoss;

/* This is a unit test that checks to see whether the driving methods using sensors work. */

@Autonomous(name = "Ross: Drive Sensors Test", group = "Ross")

public class DriveSensorsTest extends LinearOpMode {

    private HardwareRoss hardwareRoss;

    private DriveController driveController;

    @Override
    public void runOpMode() {
        hardwareRoss = new HardwareRoss(this);

        hardwareRoss.logAndShowInTelemetry("This unit test should simply move the robot forward a foot using sensors.", "");

        driveController = new DriveController(hardwareRoss);

        waitForStart();

        driveController.driveBySensors(0.1, -12, 15, 0, false);
    }

}
