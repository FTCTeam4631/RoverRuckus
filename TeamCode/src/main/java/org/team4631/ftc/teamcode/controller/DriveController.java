package org.team4631.ftc.teamcode.controller;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.team4631.ftc.teamcode.drivers.MPU9250;
import org.team4631.ftc.teamcode.hardware.HardwareRoss;
import org.team4631.ftc.teamcode.utils.PIDController;

import static java.lang.Thread.sleep;

public class DriveController extends AbstractCommonController {

    /* This controller assumes motor powers of 1 move the robot forward while motor powers of -1 move the robot backward. */

    /* Maximum forward power. */
    private static double DRIVE_MAX_FORWARD_POWER = 1;
    /* Maximum reverse power. */
    private static double DRIVE_MAX_REVERSE_POWER = -1;
    /* Power when motors are not moving. */
    private static double DRIVE_IDLE_POWER = 0;

    private static double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    private static double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    private static double WHEEL_DIAMETER_INCHES = 4.92125984;     // For figuring circumference
    private static double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private static int TICKS_PER_SEC_AT_MAX_POWER_LEFT = 2500;
    private static int TICKS_PER_SEC_AT_MAX_POWER_RIGHT = 2500;

    private static final double     HEADING_THRESHOLD       = 2;      // As tight as we can make it with an integer gyro
    private static final double     P_TURN_COEFF            = 0.05;     // Larger is more responsive, but also less stable
    private static final double     P_DRIVE_COEFF           = 0.07;     // Larger is more responsive, but also less stable

    private static String DRIVE_LEFT_FRONT_MOTOR_NAME ="leftFrontMotor";
    private static String DRIVE_LEFT_BACK_MOTOR_NAME = "leftBackMotor";
    private static String DRIVE_RIGHT_FRONT_MOTOR_NAME = "rightFrontMotor";
    private static String DRIVE_RIGHT_BACK_MOTOR_NAME = "rightBackMotor";
    private static String FIRST_HUB_IMU_NAME = "firstHubIMU";
    private static String SECOND_HUB_IMU_NAME = "secondHubIMU";

    private static DcMotor.Direction leftFrontMotorDirection = DcMotor.Direction.FORWARD;
    private static DcMotor.Direction leftBackMotorDirection = DcMotor.Direction.FORWARD;
    private static DcMotor.Direction rightFrontMotorDirection = DcMotor.Direction.REVERSE;
    private static DcMotor.Direction rightBackMotorDirection = DcMotor.Direction.REVERSE;

    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightBackMotor;
    private DcMotor leftSideEncoderMotor; // Reference to the left drive train motor which has an encoder (configurable).
    private DcMotor rightSideEncoderMotor; // Reference to the right drive train motor which has an encoder (configurable).
    private BNO055IMU firstHubIMU;
    private BNO055IMU secondHubIMU;
    private MPU9250 externalMPU;

    private Orientation firstLastAngles = new Orientation();
    private double firstGlobalAngle;
    private Orientation secondLastAngles = new Orientation();
    private double secondGlobalAngle;

    private ElapsedTime runtime = new ElapsedTime();

    public DriveController(HardwareRoss hardwareRoss) {
        super(hardwareRoss);

        /* Reset needed hardware. */
        reset();
    }

    public DcMotor getLeftFrontMotor() {
        return leftFrontMotor;
    }

    public DcMotor getLeftBackMotor() {
        return leftBackMotor;
    }

    public DcMotor getRightFrontMotor() {
        return rightFrontMotor;
    }

    public DcMotor getRightBackMotor() {
        return rightBackMotor;
    }

    public DcMotor getLeftSideEncoderMotor() {
        return leftSideEncoderMotor;
    }

    public DcMotor getRightSideEncoderMotor() {
        return rightSideEncoderMotor;
    }

    public BNO055IMU getFirstHubIMU() { return firstHubIMU; }
    public BNO055IMU getSecondHubIMU() { return secondHubIMU; }
    public MPU9250 getExternalMPU() { return externalMPU; }

    /* Driving methods. */

    public void setDriveTrainPowerForward(){
        setDriveTrainPower(DRIVE_MAX_FORWARD_POWER, DRIVE_MAX_FORWARD_POWER, DRIVE_MAX_FORWARD_POWER, DRIVE_MAX_FORWARD_POWER);
    }

    public void setDriveTrainPowerReverse(){
        setDriveTrainPower(DRIVE_MAX_REVERSE_POWER, DRIVE_MAX_REVERSE_POWER, DRIVE_MAX_REVERSE_POWER, DRIVE_MAX_REVERSE_POWER);
    }

    public void setDriveTrainPower(double leftFrontMotor, double leftBackMotor, double rightFrontMotor, double rightBackMotor){
        getLeftFrontMotor().setPower(leftFrontMotor);
        getLeftBackMotor().setPower(leftBackMotor);
        getRightFrontMotor().setPower(rightFrontMotor);
        getRightBackMotor().setPower(rightBackMotor);
    }

    public void setDriveTrainIdle(){
        setDriveTrainPower(DRIVE_IDLE_POWER, DRIVE_IDLE_POWER, DRIVE_IDLE_POWER, DRIVE_IDLE_POWER);
    }

    /* Rotation methods. */

    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle() {
        firstLastAngles = firstHubIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        firstGlobalAngle = 0;

        secondLastAngles = secondHubIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        secondGlobalAngle = 0;
    }

    public void calculateAngleData() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation firstAngles = firstHubIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Orientation secondAngles = secondHubIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double firstDeltaAngle = firstAngles.firstAngle - firstLastAngles.firstAngle;
        double secondDeltaAngle = secondAngles.firstAngle - secondLastAngles.firstAngle;

        if (firstDeltaAngle < -180)
            firstDeltaAngle += 360;
        else if (firstDeltaAngle > 180)
            firstDeltaAngle -= 360;

        if (secondDeltaAngle < -180)
            secondDeltaAngle += 360;
        else if (secondDeltaAngle > 180)
            secondDeltaAngle -= 360;

        firstGlobalAngle += firstDeltaAngle;
        secondGlobalAngle += secondDeltaAngle;

        firstLastAngles = firstAngles;
        secondLastAngles = secondAngles;
    }

    /**
     * Get current cumulative angle rotation from last reset without averaging two expansion hub IMUs.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    public double getAngleNoMean() {
        calculateAngleData();
        return firstGlobalAngle;
    }

    /**
     * Get current cumulative angle rotation from last reset with averaging two expansion hub IMUs.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    public double getAngleMean() {
        calculateAngleData();
        return (firstGlobalAngle + secondGlobalAngle) / 2;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param angle Degrees to turn, + is left - is right
     */
    public void rotate(int angle, double power, boolean averageGyros) {
        // restart imu angle tracking.
        resetAngle();

        // keep looping while we are still active, and not on heading.
        while (hardwareRoss.getLinearOpMode().opModeIsActive() && !onHeading(power, angle, P_TURN_COEFF, averageGyros)) {
            // Update telemetry & Allow time for other processes to run.
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime, boolean averageGyros) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (hardwareRoss.getLinearOpMode().opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF, averageGyros);
        }

        // Stop all motion;
        setDriveTrainIdle();
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff, boolean averageGyros) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle, averageGyros);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        setDriveTrainPower(leftSpeed, leftSpeed, rightSpeed, rightSpeed);

        // Display it for the driver.
        hardwareRoss.clearTelemetry();
        hardwareRoss.addToTelemetry("Target: ", angle);
        hardwareRoss.addToTelemetry("Error/Steer: ", Double.toString(error) + ", " + Double.toString(steer));
        hardwareRoss.addToTelemetry("Left speed/Right Speed: ", Double.toString(leftSpeed) + ", " + Double.toString(rightSpeed));
        hardwareRoss.updateTelemetry();

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle, boolean averageGyros) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - (averageGyros ? getAngleMean() : getAngleNoMean());
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    /* Driving methods. */
    public void driveByTime(double power, double time) {
        if (power < 0 || power > 1) return;

        setDriveTrainPower(power, power, power, power);

        runtime.reset();
        while (runtime.seconds() < time && hardwareRoss.getLinearOpMode().opModeIsActive()) { }

        setDriveTrainIdle();
    }

    public void setVelocitySetpoint(double leftInchesPerSec, double rightInchesPerSec, double timeStepS) {
        double leftInches = leftInchesPerSec * timeStepS;
        double rightInches = rightInchesPerSec * timeStepS;

        double leftTicks = leftInches * COUNTS_PER_INCH;
        double rightTicks = rightInches * COUNTS_PER_INCH;

        double ticksPerTimeStepLeft = TICKS_PER_SEC_AT_MAX_POWER_LEFT * timeStepS;
        double ticksPerTimeStepRight = TICKS_PER_SEC_AT_MAX_POWER_RIGHT * timeStepS;

        double leftSpeed = leftTicks / ticksPerTimeStepLeft;
        double rightSpeed = rightTicks / ticksPerTimeStepRight;

        double leftAngularRotation = 360 * (leftTicks / COUNTS_PER_MOTOR_REV);
        double rightAngularRotation = 360 * (rightTicks / COUNTS_PER_MOTOR_REV);

        hardwareRoss.clearTelemetry();
        hardwareRoss.addToTelemetry("Left velocity: ", leftInchesPerSec);
        hardwareRoss.addToTelemetry("Right velocity: ", rightInchesPerSec);
        hardwareRoss.addToTelemetry("Left inches: ", leftInches);
        hardwareRoss.addToTelemetry("Right inches: ", rightInches);
        hardwareRoss.addToTelemetry("Left ticks: ", leftTicks);
        hardwareRoss.addToTelemetry("Right ticks: ", rightTicks);
        hardwareRoss.updateTelemetry();

        setDriveTrainPower(leftSpeed, leftSpeed, rightSpeed, rightSpeed);

        int startingLeftTicks = leftSideEncoderMotor.getCurrentPosition();
        int startingRightTicks = rightSideEncoderMotor.getCurrentPosition();

        while ((leftSideEncoderMotor.getCurrentPosition() < startingLeftTicks + (int)leftTicks)
                || (rightSideEncoderMotor.getCurrentPosition() < startingRightTicks + (int)rightTicks)) {
        }
    }

    public void driveBySensors(double power, double inches, double timeoutS, double angle, boolean averageGyros) {
        resetAngle();

        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        /* The power given to this function should always be positive.
         * However, the actual power given to the motors may be negative
         * because we may want to forward or backward n inches
         * (direction determined by the sign of inches).
         */
        double motorPowers = Math.abs(power) * Math.signum(inches);

        int ticks = (int)Math.round(inches * COUNTS_PER_INCH);

        leftSideEncoderMotor.setTargetPosition(leftSideEncoderMotor.getCurrentPosition() + ticks);

        // Turn On RUN_TO_POSITION
        leftSideEncoderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setDriveTrainPower(motorPowers, motorPowers, motorPowers, motorPowers);

        // reset the timeout time and start motion.
        runtime.reset();

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (hardwareRoss.getLinearOpMode().opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (leftSideEncoderMotor.isBusy())) {
            // adjust relative speed based on heading error.
            error = getError(angle, averageGyros);
            steer = getSteer(error, P_DRIVE_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed ?
            /*
            if (inches < 0)
                steer *= -1.0;
            */

            leftSpeed = motorPowers - steer;
            rightSpeed = motorPowers + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            setDriveTrainPower(leftSpeed, leftSpeed, rightSpeed, rightSpeed);

            hardwareRoss.clearTelemetry();
            hardwareRoss.addToTelemetry("Target ticks: ", ticks);
            hardwareRoss.addToTelemetry("Actual ticks: ", leftSideEncoderMotor.getCurrentPosition());
            hardwareRoss.addToTelemetry("Error/Steer: ", Double.toString(error) + ", " + Double.toString(steer));
            hardwareRoss.addToTelemetry("Left speed/Right speed: ", Double.toString(leftSpeed) + ", " + Double.toString(rightSpeed));
            hardwareRoss.updateTelemetry();
        }

        // Stop all motion;
        setDriveTrainIdle();

        // Turn off RUN_TO_POSITION
        leftSideEncoderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetGyro(BNO055IMU imu) {
        hardwareRoss.logAndShowInTelemetry("Resetting drive gyro.", "");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);


        /* Remap axes of the IMU (our Expansion Hub is mounted vertically) by using custom hardware writes. */
        byte AXIS_MAP_CONFIG_BYTE = 0x6; /* This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes. */
        byte AXIS_MAP_SIGN_BYTE = 0x1; /* This is what to write to the AXIS_MAP_SIGN register to negate the z axis. */

        /* Need to be in CONFIG mode to write to registers. */
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);

        try {
            sleep(100); /* Changing modes requires a delay before doing anything else. */
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        hardwareRoss.logAndShowInTelemetry("Remapping drive gyro axes.", "");

        /* Write to the AXIS_MAP_CONFIG register. */
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0x0F);

        /* Write to the AXIS_MAP_SIGN register. */
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);

        /* Need to change back into the IMU mode to use the gyro. */
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);

        try {
            sleep(100); /* Changing modes again requires a delay. */
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        hardwareRoss.logAndShowInTelemetry("Finished remapping drive gyro axes.", "");

        hardwareRoss.logAndShowInTelemetry("Calibrating drive gyro.", "");

        while (!hardwareRoss.getLinearOpMode().isStopRequested() && !imu.isGyroCalibrated())
        {
            try {
                sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            hardwareRoss.getLinearOpMode().idle();
        }

        hardwareRoss.logAndShowInTelemetry("Finished calibrating drive gyro.", "");
    }

    public void resetMotor(DcMotor motor, DcMotor.Direction direction, boolean useEncoder) {
        if (useEncoder) {
            motor.setDirection(direction);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            motor.setDirection(direction);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0);
        }
    }

    public void reset() {
        /* Initialize needed hardware. */
        hardwareRoss.logAndShowInTelemetry("Resetting drive hardware.", "");

        leftFrontMotor = hardwareRoss.get(DcMotor.class, DRIVE_LEFT_FRONT_MOTOR_NAME);
        leftBackMotor = hardwareRoss.get(DcMotor.class, DRIVE_LEFT_BACK_MOTOR_NAME);
        rightFrontMotor = hardwareRoss.get(DcMotor.class, DRIVE_RIGHT_FRONT_MOTOR_NAME);
        rightBackMotor = hardwareRoss.get(DcMotor.class, DRIVE_RIGHT_BACK_MOTOR_NAME);
        firstHubIMU = hardwareRoss.get(BNO055IMU.class, FIRST_HUB_IMU_NAME);
        secondHubIMU = hardwareRoss.get(BNO055IMU.class, SECOND_HUB_IMU_NAME);

        /* Set default configurations. */
        /* Reverse right motors and keep left motors on forward because they are on opposite sides. */
        resetMotor(leftFrontMotor, leftFrontMotorDirection, false);
        resetMotor(leftBackMotor, leftBackMotorDirection, true);
        resetMotor(rightFrontMotor, rightFrontMotorDirection, false);
        resetMotor(rightBackMotor, rightBackMotorDirection, true);

        leftSideEncoderMotor = leftBackMotor;
        rightSideEncoderMotor = rightBackMotor;

        resetGyro(firstHubIMU);

        // TODO: Reset second gyro as well
        // resetGyro(secondHubIMU);

        setDriveTrainIdle();
    }

}
