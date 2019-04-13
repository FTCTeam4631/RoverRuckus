package org.team4631.ftc.teamcode.hardware;

import android.support.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Field;


public class HardwareRoss {

    private LinearOpMode linearOpMode;

    public HardwareRoss(LinearOpMode linearOpMode) {
        super();

        this.linearOpMode = linearOpMode;
    }

    public LinearOpMode getLinearOpMode() { return linearOpMode; }
    public HardwareMap getHardwareMap() { return linearOpMode.hardwareMap; }
    public Telemetry getTelemetry() { return linearOpMode.telemetry; }

    public void setLinearOpMode(LinearOpMode linearOpMode) { this.linearOpMode = linearOpMode; }

    public <T> T get(Class<? extends T> classOrInterface, String deviceName) {
        logAndShowInTelemetry("Finding Device with name: " + deviceName + " Type: " + classOrInterface.getName(), "");
        T device = getHardwareMap().get(classOrInterface, deviceName);
        logAndShowInTelemetry("Device Found with: " + deviceName + " Type: " + classOrInterface.getName(), "");
        return device;
    }

    public @Nullable
    <T> T tryGet(Class<? extends T> classOrInterface, String deviceName) {
        logAndShowInTelemetry("Finding Device: " + deviceName + " Type: " + classOrInterface.getName(), "");
        T device = getHardwareMap().tryGet(classOrInterface, deviceName);
        if (device == null) {
            logAndShowInTelemetry("No Device found with name: " + deviceName + " Type: " + classOrInterface.getName(), "");
        }
        logAndShowInTelemetry("Device Found with name: " + deviceName + " Type: " + classOrInterface.getName(), "");
        return device;
    }

    public void logDebug(String message) {
        RobotLog.d(message);
    }

    public void logInfo(String message) { RobotLog.i(message); }

    public void clearTelemetry() {
        getTelemetry().clearAll();
    }

    public<T> void addToTelemetry(String message, T o) {
        if(o == null) {
            getTelemetry().addLine(message);
        } else {
            getTelemetry().addData(message, o);
        }
    }

    public void updateTelemetry() {
        getTelemetry().update();
    }

    public <T> void logAndShowInTelemetry(String message, T o) {
        addToTelemetry(message, o);

        if(o == null) {
            logDebug(message);
        } else {
            logDebug(message + HardwareRoss.toString(o));
        }

        updateTelemetry();
    }

    public static <T> String toString(T o)  {
        StringBuilder stringBuilder = new StringBuilder(1024);
        stringBuilder.append(o.getClass().getName()).append(" [ ");

        for(Field field: o.getClass().getDeclaredFields()) {
            field.setAccessible(true);
            stringBuilder.append(field.getName()).append("-");
            stringBuilder.append(field.getType()).append(":");
            try {
                stringBuilder.append(field.get(o)).append(" ]");
            } catch (IllegalAccessException e) {
                RobotLog.d(e.getMessage());
            }
        }

        return stringBuilder.toString();
    }

}
