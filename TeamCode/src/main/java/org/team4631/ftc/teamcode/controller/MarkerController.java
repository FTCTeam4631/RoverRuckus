package org.team4631.ftc.teamcode.controller;

import com.qualcomm.robotcore.hardware.Servo;

import org.team4631.ftc.teamcode.hardware.HardwareRoss;

public class MarkerController extends AbstractCommonController {

    public static double MARKER_HOLD_POSITION = 0.0;
    public static double MARKER_RELEASE_POSITION = 1.0;

    public static String MARKER_SERVO_NAME = "markerServo";

    private Servo markerServo;

    public MarkerController(HardwareRoss hardwareRoss) {
        super(hardwareRoss);

        markerServo = hardwareRoss.get(Servo.class, MARKER_SERVO_NAME);
    }

    public Servo getMarkerServo() {
        return markerServo;
    }

    public void holdMarker() {
        markerServo.setPosition(MARKER_HOLD_POSITION);
    }

    public void releaseMarker() {
        markerServo.setPosition(MARKER_RELEASE_POSITION);
    }

    @Override
    public void reset() {
        markerServo = hardwareRoss.get(Servo.class, MARKER_SERVO_NAME);
        holdMarker();
    }

}
