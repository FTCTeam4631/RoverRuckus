package org.team4631.ftc.teamcode.controller;

import org.team4631.ftc.teamcode.hardware.HardwareRoss;

import java.lang.reflect.Field;

public abstract class AbstractCommonController {

    public HardwareRoss hardwareRoss;

    public HardwareRoss getHardwareRoss() {
        return hardwareRoss;
    }

    public void setHardwareRoss(HardwareRoss hardwareRoss) {
        this.hardwareRoss = hardwareRoss;
    }


    public AbstractCommonController(HardwareRoss hardwareRoss) {
        super ();
        this.hardwareRoss = hardwareRoss;
    }

    public abstract void reset();

    @Override
    public String toString() {
        StringBuilder stringBuilder = new StringBuilder(1024);
        try
        {
            stringBuilder.append(getClass().getName()).append(" [ ");
            for(Field field: getClass().getDeclaredFields()) {
                field.setAccessible(true);
                stringBuilder.append(field.getName()).append("-");
                stringBuilder.append(field.getType()).append(":");
                stringBuilder.append(field.get(this)).append(" ]");
            }
        } catch (IllegalAccessException accessExpcetion) {
            hardwareRoss.logAndShowInTelemetry("Exception when printing toString", accessExpcetion);
        }

        return stringBuilder.toString();
    }
}
