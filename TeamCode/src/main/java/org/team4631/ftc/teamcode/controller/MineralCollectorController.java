package org.team4631.ftc.teamcode.controller;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.team4631.ftc.teamcode.hardware.HardwareRoss;

public class MineralCollectorController extends AbstractCommonController {

    private static double EXTENDER_LIFT_MOTOR_RAISE_POWER = -0.5;
    private static double EXTENDER_LIFT_MOTOR_LOWER_POWER = 0.5;
    private static double EXTENDER_LIFT_MOTOR_IDLE_POWER = 0.0;

    private static double EXTENDER_MOTOR_OUT_POWER = 0.75;
    private static double EXTENDER_MOTOR_IN_POWER = -0.75;
    private static double EXTENDER_MOTOR_IDLE_POWER = 0.0;

    private static double INTAKE_SERVO_OUT_POWER = 1.0;
    private static double INTAKE_SERVO_IN_POWER = -1.0;
    private static double INTAKE_SERVO_IDLE_POWER = 0.0;

    private static double TILT_SERVO_FLAT_POSITION = 0.0;
    private static double TILT_SERVO_UPRIGHT_POSITION = 1.0;

    private static double STAND_SERVO_HOLD_POSITION = 0.0;
    private static double STAND_SERVO_RELEASE_POSITION = 1.0;

    private static String EXTENDER_LIFT_MOTOR_NAME = "extenderLiftMotor";
    private static String EXTENDER_MOTOR_NAME = "extenderMotor";
    private static String INTAKE_SERVO_NAME = "intakeServo";
    private static String TILT_SERVO_NAME = "tiltServo";
    private static String STAND_SERVO_NAME = "standServo";

    private DcMotor extenderLiftMotor;
    private DcMotor extenderMotor;
    private CRServo intakeServo;
    private Servo tiltServo;
    private Servo standServo;

    public MineralCollectorController(HardwareRoss hardwareRoss) {
        super(hardwareRoss);

        reset();
    }

    @Override
    public void reset() {
        extenderLiftMotor = hardwareRoss.get(DcMotor.class, EXTENDER_LIFT_MOTOR_NAME);
        extenderMotor = hardwareRoss.get(DcMotor.class, EXTENDER_MOTOR_NAME);
        intakeServo = hardwareRoss.get(CRServo.class, INTAKE_SERVO_NAME);
        tiltServo = hardwareRoss.get(Servo.class, TILT_SERVO_NAME);
        standServo = hardwareRoss.get(Servo.class, STAND_SERVO_NAME);

        extenderLiftMotor.setDirection(DcMotor.Direction.FORWARD);
        extenderLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extenderLiftMotor.setPower(0);
        extenderMotor.setDirection(DcMotor.Direction.FORWARD);
        extenderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extenderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extenderMotor.setPower(0);

        intakeServo.setDirection(CRServo.Direction.REVERSE);
        intakeServo.setPower(0);

        tiltServo.setPosition(TILT_SERVO_FLAT_POSITION);

        standServo.setPosition(STAND_SERVO_HOLD_POSITION);
    }

    public void setExtenderLiftMotorRaise() {
        extenderLiftMotor.setPower(EXTENDER_LIFT_MOTOR_RAISE_POWER);
    }

    public void setExtenderLiftMotorLower() {
        extenderLiftMotor.setPower(EXTENDER_LIFT_MOTOR_LOWER_POWER);
    }

    public void setExtenderLiftMotorIdle() {
        extenderLiftMotor.setPower(EXTENDER_LIFT_MOTOR_IDLE_POWER);
    }

    public void setExtenderMotorOut() {
        extenderMotor.setPower(EXTENDER_MOTOR_OUT_POWER);
    }

    public void setExtenderMotorIn() {
        extenderMotor.setPower(EXTENDER_MOTOR_IN_POWER);
    }

    public void setExtenderMotorIdle() {
        extenderMotor.setPower(EXTENDER_MOTOR_IDLE_POWER);
    }

    public void setIntakeMotorOut() {
        intakeServo.setPower(INTAKE_SERVO_OUT_POWER);
    }

    public void setIntakeMotorIn() {
        intakeServo.setPower(INTAKE_SERVO_IN_POWER);
    }

    public void setIntakeMotorIdle() {
        intakeServo.setPower(INTAKE_SERVO_IDLE_POWER);
    }

    public void setTiltServoFlat() { tiltServo.setPosition(TILT_SERVO_FLAT_POSITION); }

    public void setTiltServoUpright() { tiltServo.setPosition(TILT_SERVO_UPRIGHT_POSITION); }

    public void setStandServoHold() { standServo.setPosition(STAND_SERVO_HOLD_POSITION); }

    public void setStandServoRelease() { standServo.setPosition(STAND_SERVO_RELEASE_POSITION); }

}
