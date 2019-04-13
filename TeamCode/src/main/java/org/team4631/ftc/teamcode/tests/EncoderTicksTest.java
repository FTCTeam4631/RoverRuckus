package org.team4631.ftc.teamcode.tests;

/* This is a unit test that checks how many ticks the left and right encoders have went through while running at full power for 1 second. */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.team4631.ftc.teamcode.controller.DriveController;
import org.team4631.ftc.teamcode.hardware.HardwareRoss;

@Autonomous(name = "Ross: Encoder Ticks Test", group = "Ross")
public class EncoderTicksTest extends LinearOpMode {

    private HardwareRoss hardwareRoss;

    private DriveController driveController;

    @Override
    public void runOpMode() {
        hardwareRoss = new HardwareRoss(this);

        driveController = new DriveController(hardwareRoss);

        waitForStart();

        int previousLeftTicks = driveController.getLeftSideEncoderMotor().getCurrentPosition();
        int previousRightTicks = driveController.getRightSideEncoderMotor().getCurrentPosition();

        driveController.driveByTime(1.0, 1.0);

        int currentLeftTicks = driveController.getLeftSideEncoderMotor().getCurrentPosition();
        int currentRightTicks = driveController.getRightSideEncoderMotor().getCurrentPosition();

        hardwareRoss.clearTelemetry();
        hardwareRoss.addToTelemetry("Left delta ticks: ", currentLeftTicks - previousLeftTicks);
        hardwareRoss.addToTelemetry("Right delta ticks: ", currentRightTicks - previousRightTicks);
        hardwareRoss.updateTelemetry();
    }

}
