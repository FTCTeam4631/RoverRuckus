package org.team4631.ftc.teamcode.mission;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.team4631.ftc.teamcode.controller.LatchController;
import org.team4631.ftc.teamcode.controller.LauncherController;
import org.team4631.ftc.teamcode.hardware.HardwareRoss;
import org.team4631.ftc.teamcode.utils.PIDController;

public class LandingManager {

    private static final double LOWERED_DISTANCE_FROM_GROUND = 4.0625;
    private static final double LOWERED_DISTANCE_FROM_GROUND_TOLERANCE = 0.1;

    private HardwareRoss hardwareRoss;
    private LauncherController launcherController;
    private LatchController latchController;
    private ElapsedTime runtime;
    private PIDController launcherLoweringPID;
    private PIDController launcherRaisingPID;

    public LandingManager(HardwareRoss hardwareRoss) {
        this.hardwareRoss = hardwareRoss;
        this.launcherController = new LauncherController(hardwareRoss);
        this.latchController = new LatchController(hardwareRoss);
        this.runtime = new ElapsedTime();

        this.latchController.lockLatch();
    }

    public void landRobotByTime() {
        hardwareRoss.logAndShowInTelemetry("Landing sequence started.", "");

        lowerLauncherByTime();

        unlockLatch();

        raiseLauncher();
    }

    public void landRobotBySensor() {
        hardwareRoss.logAndShowInTelemetry("Landing sequence started.", "");

        lowerLauncherBySensor();

        unlockLatch();

        raiseLauncher();
    }

    public void lowerLauncherByTime() {
        launcherController.lowerLauncher();

        runtime.reset();
        while (runtime.seconds() < 5.0 && hardwareRoss.getLinearOpMode().opModeIsActive()) {
            hardwareRoss.logAndShowInTelemetry("Lowering launcher...", "");
        }

        launcherController.setLauncherIdle();
    }

    public void lowerLauncherBySensor() {
        launcherController.lowerLauncher();

        double distanceToGround = 0.0;

        do {
            distanceToGround = launcherController.getLauncherDistanceSensor().getDistance(DistanceUnit.CM);
            hardwareRoss.logAndShowInTelemetry("Distance to ground (cm): ", distanceToGround);
        } while (distanceToGround > LOWERED_DISTANCE_FROM_GROUND - LOWERED_DISTANCE_FROM_GROUND_TOLERANCE);

        launcherController.setLauncherIdle();
    }

    public void raiseLauncher() {
        launcherController.raiseLauncher();

        runtime.reset();
        while (runtime.seconds() < 5.0 && hardwareRoss.getLinearOpMode().opModeIsActive()) {
            hardwareRoss.logAndShowInTelemetry("Raising launcher...", "");
        }

        launcherController.setLauncherIdle();
    }

    public void unlockLatch() {
        latchController.unlockLatch();

        runtime.reset();
        while (runtime.seconds() < 1.0 && hardwareRoss.getLinearOpMode().opModeIsActive()) {
            hardwareRoss.logAndShowInTelemetry("Unlocking latch...", "");
        }
    }

}