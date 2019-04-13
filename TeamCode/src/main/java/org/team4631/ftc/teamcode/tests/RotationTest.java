package org.team4631.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.team4631.ftc.teamcode.controller.DriveController;
import org.team4631.ftc.teamcode.hardware.HardwareRoss;

/* This is a unit test that checks to see whether the rotation methods work. */

@Autonomous(name = "Ross: Rotation Test", group = "Ross")

public class RotationTest extends LinearOpMode {

    private HardwareRoss hardwareRoss;

    private DriveController driveController;

    @Override
    public void runOpMode() {
        hardwareRoss = new HardwareRoss(this);

        hardwareRoss.logAndShowInTelemetry("This unit test should simply turn the robot 90 degrees to the left on a fixed point.", "");

        driveController = new DriveController(hardwareRoss);

        waitForStart();

        driveController.resetAngle();

        while (opModeIsActive()) {
            hardwareRoss.logAndShowInTelemetry("Gyro angle: ", driveController.getAngleNoMean());
        }
    }

}
