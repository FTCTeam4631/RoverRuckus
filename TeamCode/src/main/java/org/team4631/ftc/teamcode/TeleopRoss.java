package org.team4631.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


import org.team4631.ftc.teamcode.controller.DriveController;
import org.team4631.ftc.teamcode.controller.LatchController;
import org.team4631.ftc.teamcode.controller.LauncherController;
import org.team4631.ftc.teamcode.controller.MineralCollectorController;
import org.team4631.ftc.teamcode.hardware.HardwareRoss;
import org.team4631.ftc.teamcode.mission.LandingManager;

@TeleOp(name = "Ross: Teleop", group = "Ross")
public class TeleopRoss extends LinearOpMode {

    private HardwareRoss hardwareRoss;

    private DriveController driveController;
    private LauncherController launcherController;
    private LatchController latchController;
    private MineralCollectorController mineralCollectorController;

    private static double drivetrainServosSpeed = 0.5;
    private static double leftTriggerPushThreshold = 0.5;
    private static double rightTriggerPushThreshold = 0.5;

    private double drivetrainSpeedPercentage = 0.0;

    private boolean lastDPadButtonUp = false;
    private boolean lastDPadButtonDown = false;
    private boolean currentDPadButtonUp = false;
    private boolean currentDPadButtonDown = false;

    @Override
    public void runOpMode() throws InterruptedException {
        hardwareRoss = new HardwareRoss(this);

        driveController = new DriveController(hardwareRoss);
        launcherController = new LauncherController(hardwareRoss);
        latchController = new LatchController(hardwareRoss);
        mineralCollectorController = new MineralCollectorController(hardwareRoss);

        hardwareRoss.logAndShowInTelemetry("Started teleop.", "");

        waitForStart();

        drivetrainSpeedPercentage = 0.75;

        while (opModeIsActive()) {
            lastDPadButtonUp = currentDPadButtonUp;
            currentDPadButtonUp = gamepad1.dpad_up;

            lastDPadButtonDown = currentDPadButtonDown;
            currentDPadButtonDown = gamepad1.dpad_down;

            if (currentDPadButtonUp != lastDPadButtonUp && currentDPadButtonUp) {
                drivetrainSpeedPercentage += 0.05;
            }

            if (currentDPadButtonDown != lastDPadButtonDown && currentDPadButtonDown) {
                drivetrainSpeedPercentage -= 0.05;
            }

            if (gamepad1.dpad_left) {
                drivetrainSpeedPercentage = 0.3;
            }

            if (gamepad1.dpad_right) {
                drivetrainSpeedPercentage = 0.7;
            }

            lastDPadButtonUp = gamepad1.dpad_up;
            lastDPadButtonDown = gamepad1.dpad_down;

            if (drivetrainSpeedPercentage < 0.3) {
                drivetrainSpeedPercentage = 0.3;
            }

            if (drivetrainSpeedPercentage > 1.0) {
                drivetrainSpeedPercentage = 1.0;
            }

            hardwareRoss.logAndShowInTelemetry("Drive train speed percentage: ", drivetrainSpeedPercentage);

            if (gamepad2.left_bumper) {
                mineralCollectorController.setExtenderLiftMotorRaise();
            } else if (gamepad2.right_bumper) {
                mineralCollectorController.setExtenderLiftMotorLower();
            } else {
                mineralCollectorController.setExtenderLiftMotorIdle();
            }

            if (gamepad2.y) {
                mineralCollectorController.setExtenderMotorOut();
            } else if (gamepad2.a) {
                mineralCollectorController.setExtenderMotorIn();
            } else {
                mineralCollectorController.setExtenderMotorIdle();
            }

            if (gamepad2.left_trigger > 0.5) {
                mineralCollectorController.setIntakeMotorOut();
            } else if (gamepad2.right_trigger > 0.5) {
                mineralCollectorController.setIntakeMotorIn();
            } else {
                mineralCollectorController.setIntakeMotorIdle();
            }

            double leftPower;
            double rightPower;

            leftPower = -gamepad1.left_stick_y * drivetrainSpeedPercentage;
            rightPower = -gamepad1.right_stick_y * drivetrainSpeedPercentage;

            driveController.setDriveTrainPower(leftPower, leftPower, rightPower, rightPower);

            /*
            if (gamepad1.right_bumper && gamepad1.left_bumper) {
                launcherController.setLauncherPower(-drivetrainServosSpeed, -drivetrainServosSpeed, -drivetrainServosSpeed, -drivetrainServosSpeed);
            } else if (gamepad1.left_trigger > leftTriggerPushThreshold
                    && gamepad1.right_trigger > rightTriggerPushThreshold) {
                launcherController.setLauncherPower(drivetrainServosSpeed, drivetrainServosSpeed, drivetrainServosSpeed, drivetrainServosSpeed);
            } else {
                launcherController.setLauncherIdle();
            }
            */

            if (gamepad1.a) {
                latchController.lockLatch();
            }

            if (gamepad1.y) {
                latchController.unlockLatch();
            }

            if (gamepad1.b) {
                mineralCollectorController.setTiltServoFlat();
            }

            if (gamepad1.x) {
                mineralCollectorController.setTiltServoUpright();
            }

            if (gamepad2.b) {
                // mineralCollectorController.setStandServoHold();
                launcherController.setLauncherPower(-drivetrainServosSpeed, -drivetrainServosSpeed, -drivetrainServosSpeed, -drivetrainServosSpeed);

            } else if (gamepad2.x) {
                // mineralCollectorController.setStandServoRelease();
                launcherController.setLauncherPower(drivetrainServosSpeed, drivetrainServosSpeed, drivetrainServosSpeed, drivetrainServosSpeed);
            } else {
                launcherController.setLauncherIdle();
            }
        }

        hardwareRoss.logAndShowInTelemetry("Stopped teleop.", "");
    }

}
