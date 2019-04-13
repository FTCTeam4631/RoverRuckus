package org.team4631.ftc.teamcode.utils;

public class Ramper {
    protected double tUp, tDown, tDuration, tMin, tMax, tValue;

    public Ramper(double tUp, double tDown, double tDuration, double tMin, double tMax) {
        this.tUp = tUp;
        this.tDown = tDown;
        this.tDuration = tDuration;
        this.tMax = tMax;
        this.tMin = tMin;
    }

    public double getRampValue (double t) {
        double rampUpSpeed, rampDownSpeed, value;

        // speed is proportional to number of encoder steps away from target
        rampDownSpeed = 1 / tDown * (tDuration - t);
        // don't go faster than 1
        rampDownSpeed = Math.signum(rampDownSpeed) * Math.min(Math.abs(rampDownSpeed), 1);
        // don't go slower than 0
        rampDownSpeed = Math.signum(rampDownSpeed) * Math.max(Math.abs(rampDownSpeed), 0);

        // speed is proportional to number of encoder steps away from 0
        rampUpSpeed = 1 / tUp * (t);
        // don't go faster than 1
        rampUpSpeed = Math.signum(rampUpSpeed) * Math.min(Math.abs(rampUpSpeed), 1);
        // don't go slower than 0
        rampUpSpeed = Math.signum(rampUpSpeed) * Math.max(Math.abs(rampUpSpeed), 0);

        // real speed
        value = rampDownSpeed * rampUpSpeed;

        value = Math.max(tMin, value);
        value = Math.min(tMax, value);
        tValue = value;
        return value;
    }

}