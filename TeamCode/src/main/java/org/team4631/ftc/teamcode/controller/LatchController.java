package org.team4631.ftc.teamcode.controller;

import com.qualcomm.robotcore.hardware.Servo;

import org.team4631.ftc.teamcode.hardware.HardwareRoss;

public class LatchController extends AbstractCommonController {

    /* Unlocked latch position. */
    public static double LATCH_UNLOCK_POSITION = 0.0;

    /* Locked latch position. */
    public static double LATCH_LOCK_POSITION = 1.0;

    private static String LATCH_SERVO_NAME = "landerLatchServo";

    private Servo landerLatchServo;

    public LatchController(HardwareRoss hardwareRoss){
        super(hardwareRoss);

        /* Reset needed hardware. */
        reset();
    }

    public Servo getLanderLatchServo() {
        return landerLatchServo;
    }

    public void setLatchPosition(double position) {
        landerLatchServo.setPosition(position);
    }

    public void unlockLatch() {
        setLatchPosition(LATCH_UNLOCK_POSITION);
    }
    public void lockLatch() {
        setLatchPosition(LATCH_LOCK_POSITION);
    }

    @Override
    public void reset() {
        hardwareRoss.logAndShowInTelemetry("Resetting latch hardware.", "");

        landerLatchServo = hardwareRoss.get(Servo.class, LATCH_SERVO_NAME);
        landerLatchServo.setDirection(Servo.Direction.FORWARD);
    }

}
