package org.team4631.ftc.teamcode.controller;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.team4631.ftc.teamcode.hardware.HardwareRoss;

public class LauncherController extends AbstractCommonController {

    public static double LAUNCHER_LOWER = -1;
    public static double LAUNCHER_RAISE = 1;
    public static double LAUNCHER_IDLE = 0;

    private static String LEFT_FRONT_SERVO_NAME = "leftFrontServo";
    private static String LEFT_BACK_SERVO_NAME = "leftBackServo";
    private static String RIGHT_FRONT_SERVO_NAME = "rightFrontServo";
    private static String RIGHT_BACK_SERVO_NAME = "rightBackServo";
    private static String LAUNCHER_DISTANCE_SENSOR_NAME = "launcherDistanceSensor";

    private CRServo leftFrontServo;
    private CRServo leftBackServo;
    private CRServo rightFrontServo;
    private CRServo rightBackServo;

    private DistanceSensor launcherDistanceSensor;

    public LauncherController(HardwareRoss hardwareRoss) {
        super(hardwareRoss);

        /* Reset needed hardware. */
        reset();
    }

    public void setLauncherPower(double leftFrontServoPosition, double leftBackServoPosition, double rightFrontServoPosition, double rightBackServoPosition){
        leftFrontServo.setPower(leftFrontServoPosition);
        leftBackServo.setPower(leftBackServoPosition);
        rightFrontServo.setPower(rightFrontServoPosition);
        rightBackServo.setPower(rightBackServoPosition);
    }

    public void lowerLauncher() {
        setLauncherPower(LAUNCHER_LOWER, LAUNCHER_LOWER, LAUNCHER_LOWER, LAUNCHER_LOWER);
    }

    public void raiseLauncher() {
        setLauncherPower(LAUNCHER_RAISE, LAUNCHER_RAISE, LAUNCHER_RAISE, LAUNCHER_RAISE);
    }

    public void setLauncherIdle() {
        setLauncherPower(LAUNCHER_IDLE, LAUNCHER_IDLE, LAUNCHER_IDLE, LAUNCHER_IDLE);
    }

    public DistanceSensor getLauncherDistanceSensor() {
        return launcherDistanceSensor;
    }

    public void reset(){
        /* Initialize launcher hardware. */

        leftFrontServo = hardwareRoss.get(CRServo.class, LEFT_FRONT_SERVO_NAME);
        leftBackServo = hardwareRoss.get(CRServo.class,LEFT_BACK_SERVO_NAME);
        rightFrontServo = hardwareRoss.get(CRServo.class, RIGHT_FRONT_SERVO_NAME);
        rightBackServo = hardwareRoss.get(CRServo.class, RIGHT_BACK_SERVO_NAME);
        launcherDistanceSensor = hardwareRoss.get(DistanceSensor.class, LAUNCHER_DISTANCE_SENSOR_NAME);

        leftFrontServo.setDirection(CRServo.Direction.REVERSE);
        leftFrontServo.setPower(0);
        leftBackServo.setDirection(CRServo.Direction.FORWARD);
        leftBackServo.setPower(0);
        rightFrontServo.setDirection(CRServo.Direction.FORWARD);
        rightFrontServo.setPower(0);
        rightBackServo.setDirection(CRServo.Direction.REVERSE);

        setLauncherIdle();
    }

}
