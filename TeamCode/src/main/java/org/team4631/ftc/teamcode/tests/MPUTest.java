package org.team4631.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.team4631.ftc.teamcode.drivers.MPU9250;
import org.team4631.ftc.teamcode.hardware.HardwareRoss;

/* This is a unit test that checks to see whether the external I2C MPU9250 works. */

@Autonomous(name = "Ross: MPU Test", group = "Ross")
public class MPUTest extends LinearOpMode {

    private HardwareRoss hardwareRoss;

    private MPU9250 externalMPU;

    @Override
    public void runOpMode() {
        externalMPU = hardwareRoss.get(MPU9250.class, "externalMPU");

        hardwareRoss.logAndShowInTelemetry("Beginning external MPU9250 initialization.", "");

        int status = 0;

        status = externalMPU.begin();

        if (status < 0) {
            hardwareRoss.clearTelemetry();
            hardwareRoss.addToTelemetry("MPU9250 initialization was unsuccessful. Check wiring or try cycling power.", "");
            hardwareRoss.addToTelemetry("MPU9250 returned status: ", status);
            hardwareRoss.updateTelemetry();
            return;
        }

        waitForStart();

        while (opModeIsActive()) {
            hardwareRoss.clearTelemetry();
            hardwareRoss.addToTelemetry("MPU9250 accelerometer x: ", externalMPU.getAccelX_mss());
            hardwareRoss.addToTelemetry("MPU9250 accelerometer y: ", externalMPU.getAccelY_mss());
            hardwareRoss.addToTelemetry("MPU9250 accelerometer z: ", externalMPU.getAccelZ_mss());
            hardwareRoss.addToTelemetry("MPU9250 gyro x: ", externalMPU.getGyroX_rads());
            hardwareRoss.addToTelemetry("MPU9250 gyro y: ", externalMPU.getGyroY_rads());
            hardwareRoss.addToTelemetry("MPU9250 gyro z: ", externalMPU.getGyroZ_rads());
            hardwareRoss.addToTelemetry("MPU9250 magnetometer x: ", externalMPU.getMagX_uT());
            hardwareRoss.addToTelemetry("MPU9250 magnetometer y: ", externalMPU.getMagY_uT());
            hardwareRoss.addToTelemetry("MPU9250 magnetometer z: ", externalMPU.getMagZ_uT());
            hardwareRoss.updateTelemetry();
        }
    }

}
