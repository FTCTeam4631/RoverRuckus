package org.team4631.ftc.teamcode.drivers;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;

import static java.lang.Thread.sleep;

/* This driver was ported from Arduino C++ code here: https://github.com/bolderflight/MPU9250. */

@I2cSensor(name = "MPU9250 Motion Tracker", description = "MPU9250 Accelerometer, Gyro, and Compass Module", xmlTag = "MPU9250")
public class MPU9250 extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    public static final int ADDRESS_I2C_DEFAULT = 0x68;

    public enum GyroRange
    {
        GYRO_RANGE_250DPS(0),
        GYRO_RANGE_500DPS(1),
        GYRO_RANGE_1000DPS(2),
        GYRO_RANGE_2000DPS(3);

        private final int value;

        private GyroRange(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    };

    public enum AccelRange
    {
        ACCEL_RANGE_2G(0),
        ACCEL_RANGE_4G(1),
        ACCEL_RANGE_8G(2),
        ACCEL_RANGE_16G(3);

        private final int value;

        private AccelRange(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    };
    public enum DlpfBandwidth
    {
        DLPF_BANDWIDTH_184HZ(0),
        DLPF_BANDWIDTH_92HZ(1),
        DLPF_BANDWIDTH_41HZ(2),
        DLPF_BANDWIDTH_20HZ(3),
        DLPF_BANDWIDTH_10HZ(4),
        DLPF_BANDWIDTH_5HZ(5);

        private final int value;

        private DlpfBandwidth(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    };

    public enum LpAccelOdr
    {
        LP_ACCEL_ODR_0_24HZ(0),
        LP_ACCEL_ODR_0_49HZ(1),
        LP_ACCEL_ODR_0_98HZ(2),
        LP_ACCEL_ODR_1_95HZ(3),
        LP_ACCEL_ODR_3_91HZ(4),
        LP_ACCEL_ODR_7_81HZ(5),
        LP_ACCEL_ODR_15_63HZ(6),
        LP_ACCEL_ODR_31_25HZ(7),
        LP_ACCEL_ODR_62_50HZ(8),
        LP_ACCEL_ODR_125HZ(9),
        LP_ACCEL_ODR_250HZ(10),
        LP_ACCEL_ODR_500HZ(11);

        private final int value;

        private LpAccelOdr(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    };

    // track success of interacting with sensor
    private int _status;
    // buffer for reading from sensor
    private byte _buffer[];
    // data counts
    private int _axcounts,_aycounts,_azcounts;
    private int _gxcounts,_gycounts,_gzcounts;
    private int _hxcounts,_hycounts,_hzcounts;
    private int _tcounts;
    // data buffer
    private float _ax, _ay, _az;
    private float _gx, _gy, _gz;
    private float _hx, _hy, _hz;
    private float _t;
    // wake on motion
    private char _womThreshold;
    // scale factors
    private float _accelScale;
    private float _gyroScale;
    private float _magScaleX, _magScaleY, _magScaleZ;
    private final float _tempScale = 333.87f;
    private final float _tempOffset = 21.0f;
    // configuration
    private AccelRange _accelRange;
    private GyroRange _gyroRange;
    private DlpfBandwidth _bandwidth;
    private char _srd;
    // gyro bias estimation
    private int _numSamples = 100;
    private double _gxbD, _gybD, _gzbD;
    private float _gxb, _gyb, _gzb;
    // accel bias and scale factor estimation
    private double _axbD, _aybD, _azbD;
    private float _axmax, _aymax, _azmax;
    private float _axmin, _aymin, _azmin;
    private float _axb, _ayb, _azb;
    private float _axs = 1.0f;
    private float _ays = 1.0f;
    private float _azs = 1.0f;
    // magnetometer bias and scale factor estimation
    private int _maxCounts = 1000;
    private float _deltaThresh = 0.3f;
    private char _coeff = 8;
    private int _counter;
    private float _framedelta, _delta;
    private float _hxfilt, _hyfilt, _hzfilt;
    private float _hxmax, _hymax, _hzmax;
    private float _hxmin, _hymin, _hzmin;
    private float _hxb, _hyb, _hzb;
    private float _hxs = 1.0f;
    private float _hys = 1.0f;
    private float _hzs = 1.0f;
    private float _avgs;
    // transformation matrix
    /* transform the accel and gyro axes to match the magnetometer axes */
    private final int tX[] = {0,  1,  0};
    private final int tY[] = {1,  0,  0};
    private final int tZ[] = {0,  0, -1};
    // finalants
    private final float G = 9.807f;
    private final float _d2r = 3.14159265359f/180.0f;
    // MPU9250 registers
    private final char ACCEL_OUT = 0x3B;
    private final char GYRO_OUT = 0x43;
    private final char TEMP_OUT = 0x41;
    private final char EXT_SENS_DATA_00 = 0x49;
    private final char ACCEL_CONFIG = 0x1C;
    private final char ACCEL_FS_SEL_2G = 0x00;
    private final char ACCEL_FS_SEL_4G = 0x08;
    private final char ACCEL_FS_SEL_8G = 0x10;
    private final char ACCEL_FS_SEL_16G = 0x18;
    private final char GYRO_CONFIG = 0x1B;
    private final char GYRO_FS_SEL_250DPS = 0x00;
    private final char GYRO_FS_SEL_500DPS = 0x08;
    private final char GYRO_FS_SEL_1000DPS = 0x10;
    private final char GYRO_FS_SEL_2000DPS = 0x18;
    private final char ACCEL_CONFIG2 = 0x1D;
    private final char ACCEL_DLPF_184 = 0x01;
    private final char ACCEL_DLPF_92 = 0x02;
    private final char ACCEL_DLPF_41 = 0x03;
    private final char ACCEL_DLPF_20 = 0x04;
    private final char ACCEL_DLPF_10 = 0x05;
    private final char ACCEL_DLPF_5 = 0x06;
    private final char CONFIG = 0x1A;
    private final char GYRO_DLPF_184 = 0x01;
    private final char GYRO_DLPF_92 = 0x02;
    private final char GYRO_DLPF_41 = 0x03;
    private final char GYRO_DLPF_20 = 0x04;
    private final char GYRO_DLPF_10 = 0x05;
    private final char GYRO_DLPF_5 = 0x06;
    private final char SMPDIV = 0x19;
    private final char INT_PIN_CFG = 0x37;
    private final char INT_ENABLE = 0x38;
    private final char INT_DISABLE = 0x00;
    private final char INT_PULSE_50US = 0x00;
    private final char INT_WOM_EN = 0x40;
    private final char INT_RAW_RDY_EN = 0x01;
    private final char PWR_MGMNT_1 = 0x6B;
    private final char PWR_CYCLE = 0x20;
    private final char PWR_RESET = 0x80;
    private final char CLOCK_SEL_PLL = 0x01;
    private final char PWR_MGMNT_2 = 0x6C;
    private final char SEN_ENABLE = 0x00;
    private final char DIS_GYRO = 0x07;
    private final char USER_CTRL = 0x6A;
    private final char I2C_MST_EN = 0x20;
    private final char I2C_MST_CLK = 0x0D;
    private final char I2C_MST_CTRL = 0x24;
    private final char I2C_SLV0_ADDR = 0x25;
    private final char I2C_SLV0_REG = 0x26;
    private final char I2C_SLV0_DO = 0x63;
    private final char I2C_SLV0_CTRL = 0x27;
    private final char I2C_SLV0_EN = 0x80;
    private final char I2C_READ_FLAG = 0x80;
    private final char MOT_DETECT_CTRL = 0x69;
    private final char ACCEL_INTEL_EN = 0x80;
    private final char ACCEL_INTEL_MODE = 0x40;
    private final char LP_ACCEL_ODR = 0x1E;
    private final char WOM_THR = 0x1F;
    private final char WHO_AM_I = 0x75;
    // AK8963 registers
    private final char AK8963_I2C_ADDR = 0x0C;
    private final char AK8963_HXL = 0x03;
    private final char AK8963_CNTL1 = 0x0A;
    private final char AK8963_PWR_DOWN = 0x00;
    private final char AK8963_CNT_MEAS1 = 0x12;
    private final char AK8963_CNT_MEAS2 = 0x16;
    private final char AK8963_FUSE_ROM = 0x0F;
    private final char AK8963_CNTL2 = 0x0B;
    private final char AK8963_RESET = 0x01;
    private final char AK8963_ASA = 0x10;
    private final char AK8963_WHO_AM_I = 0x00;

    public MPU9250(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);

        /* Set the device client's I2C address. */
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(ADDRESS_I2C_DEFAULT));

        super.registerArmingStateCallback(false);
    }

    public int begin() {
        /* Engage connection with the MPU device. */
        this.deviceClient.engage();

        // select clock source to gyro
        if(writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0){
            return -1;
        }
        // enable I2C master mode
        if(writeRegister(USER_CTRL,I2C_MST_EN) < 0){
            return -2;
        }
        // set the I2C bus speed to 400 kHz
        if(writeRegister(I2C_MST_CTRL,I2C_MST_CLK) < 0){
            return -3;
        }
        // set AK8963 to Power Down
        writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
        // reset the MPU9250
        writeRegister(PWR_MGMNT_1,PWR_RESET);
        // wait for MPU-9250 to come back up
        delay(1);
        // reset the AK8963
        writeAK8963Register(AK8963_CNTL2,AK8963_RESET);
        // select clock source to gyro
        if(writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0){
            return -4;
        }
        // check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
        if((whoAmI() != 113)&&(whoAmI() != 115)){
            return -5;
        }
        // enable accelerometer and gyro
        if(writeRegister(PWR_MGMNT_2,SEN_ENABLE) < 0){
            return -6;
        }
        // setting accel range to 16G as default
        if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_16G) < 0){
            return -7;
        }
        _accelScale = G * 16.0f/32767.5f; // setting the accel scale to 16G
        _accelRange = AccelRange.ACCEL_RANGE_16G;
        // setting the gyro range to 2000DPS as default
        if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_2000DPS) < 0){
            return -8;
        }
        _gyroScale = 2000.0f/32767.5f * _d2r; // setting the gyro scale to 2000DPS
        _gyroRange = GyroRange.GYRO_RANGE_2000DPS;
        // setting bandwidth to 184Hz as default
        if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_184) < 0){
            return -9;
        }
        if(writeRegister(CONFIG,GYRO_DLPF_184) < 0){ // setting gyro bandwidth to 184Hz
            return -10;
        }
        _bandwidth = DlpfBandwidth.DLPF_BANDWIDTH_184HZ;
        // setting the sample rate divider to 0 as default
        if(writeRegister(SMPDIV, (char) 0x00) < 0){
            return -11;
        }
        _srd = 0;
        // enable I2C master mode
        if(writeRegister(USER_CTRL,I2C_MST_EN) < 0){
            return -12;
        }
        // set the I2C bus speed to 400 kHz
        if( writeRegister(I2C_MST_CTRL,I2C_MST_CLK) < 0){
            return -13;
        }
        // check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
        if( whoAmIAK8963() != 72 ){
            return -14;
        }
        /* get the magnetometer calibration */
        // set AK8963 to Power Down
        if(writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
            return -15;
        }
        delay(100); // long wait between AK8963 mode changes
        // set AK8963 to FUSE ROM access
        if(writeAK8963Register(AK8963_CNTL1,AK8963_FUSE_ROM) < 0){
            return -16;
        }
        delay(100); // long wait between AK8963 mode changes
        // read the AK8963 ASA registers and compute magnetometer scale factors
        readAK8963Registers(AK8963_ASA, (char) 3);
        _magScaleX = ((((float)_buffer[0]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
        _magScaleY = ((((float)_buffer[1]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
        _magScaleZ = ((((float)_buffer[2]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
        // set AK8963 to Power Down
        if(writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
            return -17;
        }
        delay(100); // long wait between AK8963 mode changes
        // set AK8963 to 16 bit resolution, 100 Hz update rate
        if(writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2) < 0){
            return -18;
        }
        delay(100); // long wait between AK8963 mode changes
        // select clock source to gyro
        if(writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0){
            return -19;
        }
        // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
        readAK8963Registers(AK8963_HXL, (char) 7);
        // estimate gyro bias
        if (calibrateGyro() < 0) {
            return -20;
        }
        // successful init, return 1
        return 1;
    }

    public int setAccelRange(AccelRange range) {
        switch(range) {
            case ACCEL_RANGE_2G: {
                // setting the accel range to 2G
                if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_2G) < 0){
                    return -1;
                }
                _accelScale = G * 2.0f/32767.5f; // setting the accel scale to 2G
                break;
            }
            case ACCEL_RANGE_4G: {
                // setting the accel range to 4G
                if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_4G) < 0){
                    return -1;
                }
                _accelScale = G * 4.0f/32767.5f; // setting the accel scale to 4G
                break;
            }
            case ACCEL_RANGE_8G: {
                // setting the accel range to 8G
                if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_8G) < 0){
                    return -1;
                }
                _accelScale = G * 8.0f/32767.5f; // setting the accel scale to 8G
                break;
            }
            case ACCEL_RANGE_16G: {
                // setting the accel range to 16G
                if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_16G) < 0){
                    return -1;
                }
                _accelScale = G * 16.0f/32767.5f; // setting the accel scale to 16G
                break;
            }
        }
        _accelRange = range;
        return 1;
    }

    public int setGyroRange(GyroRange range) {
        switch(range) {
            case GYRO_RANGE_250DPS: {
                // setting the gyro range to 250DPS
                if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_250DPS) < 0){
                    return -1;
                }
                _gyroScale = 250.0f/32767.5f * _d2r; // setting the gyro scale to 250DPS
                break;
            }
            case GYRO_RANGE_500DPS: {
                // setting the gyro range to 500DPS
                if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_500DPS) < 0){
                    return -1;
                }
                _gyroScale = 500.0f/32767.5f * _d2r; // setting the gyro scale to 500DPS
                break;
            }
            case GYRO_RANGE_1000DPS: {
                // setting the gyro range to 1000DPS
                if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_1000DPS) < 0){
                    return -1;
                }
                _gyroScale = 1000.0f/32767.5f * _d2r; // setting the gyro scale to 1000DPS
                break;
            }
            case GYRO_RANGE_2000DPS: {
                // setting the gyro range to 2000DPS
                if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_2000DPS) < 0){
                    return -1;
                }
                _gyroScale = 2000.0f/32767.5f * _d2r; // setting the gyro scale to 2000DPS
                break;
            }
        }
        _gyroRange = range;
        return 1;
    }

    public int setDlpfBandwidth(DlpfBandwidth bandwidth) {
        switch(bandwidth) {
            case DLPF_BANDWIDTH_184HZ: {
                if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_184) < 0){ // setting accel bandwidth to 184Hz
                    return -1;
                }
                if(writeRegister(CONFIG,GYRO_DLPF_184) < 0){ // setting gyro bandwidth to 184Hz
                    return -2;
                }
                break;
            }
            case DLPF_BANDWIDTH_92HZ: {
                if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_92) < 0){ // setting accel bandwidth to 92Hz
                    return -1;
                }
                if(writeRegister(CONFIG,GYRO_DLPF_92) < 0){ // setting gyro bandwidth to 92Hz
                    return -2;
                }
                break;
            }
            case DLPF_BANDWIDTH_41HZ: {
                if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_41) < 0){ // setting accel bandwidth to 41Hz
                    return -1;
                }
                if(writeRegister(CONFIG,GYRO_DLPF_41) < 0){ // setting gyro bandwidth to 41Hz
                    return -2;
                }
                break;
            }
            case DLPF_BANDWIDTH_20HZ: {
                if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_20) < 0){ // setting accel bandwidth to 20Hz
                    return -1;
                }
                if(writeRegister(CONFIG,GYRO_DLPF_20) < 0){ // setting gyro bandwidth to 20Hz
                    return -2;
                }
                break;
            }
            case DLPF_BANDWIDTH_10HZ: {
                if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_10) < 0){ // setting accel bandwidth to 10Hz
                    return -1;
                }
                if(writeRegister(CONFIG,GYRO_DLPF_10) < 0){ // setting gyro bandwidth to 10Hz
                    return -2;
                }
                break;
            }
            case DLPF_BANDWIDTH_5HZ: {
                if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_5) < 0){ // setting accel bandwidth to 5Hz
                    return -1;
                }
                if(writeRegister(CONFIG,GYRO_DLPF_5) < 0){ // setting gyro bandwidth to 5Hz
                    return -2;
                }
                break;
            }
        }
        _bandwidth = bandwidth;
        return 1;
    }

    public int setSrd(char srd) {
        /* setting the sample rate divider to 19 to facilitate setting up magnetometer */
        if(writeRegister(SMPDIV, (char) 19) < 0){ // setting the sample rate divider
            return -1;
        }
        if(srd > 9){
            // set AK8963 to Power Down
            if(writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
                return -2;
            }
            delay(100); // long wait between AK8963 mode changes
            // set AK8963 to 16 bit resolution, 8 Hz update rate
            if(writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS1) < 0){
                return -3;
            }
            delay(100); // long wait between AK8963 mode changes
            // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
            readAK8963Registers(AK8963_HXL, (char) 7);
        } else {
            // set AK8963 to Power Down
            if(writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
                return -2;
            }
            delay(100); // long wait between AK8963 mode changes
            // set AK8963 to 16 bit resolution, 100 Hz update rate
            if(writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2) < 0){
                return -3;
            }
            delay(100); // long wait between AK8963 mode changes
            // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
            readAK8963Registers(AK8963_HXL, (char) 7);
        }
        /* setting the sample rate divider */
        if(writeRegister(SMPDIV,srd) < 0){ // setting the sample rate divider
            return -4;
        }
        _srd = srd;
        return 1;
    }

    public int enableDataReadyInterrupt() {
        /* setting the interrupt */
        if (writeRegister(INT_PIN_CFG,INT_PULSE_50US) < 0){ // setup interrupt, 50 us pulse
            return -1;
        }
        if (writeRegister(INT_ENABLE,INT_RAW_RDY_EN) < 0){ // set to data ready
            return -2;
        }
        return 1;
    }

    public int disableDataReadyInterrupt() {
        if(writeRegister(INT_ENABLE,INT_DISABLE) < 0){ // disable interrupt
            return -1;
        }
        return 1;
    }

    public int enableWakeOnMotion(float womThresh_mg, LpAccelOdr odr) {
        // set AK8963 to Power Down
        writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
        // reset the MPU9250
        writeRegister(PWR_MGMNT_1,PWR_RESET);
        // wait for MPU-9250 to come back up
        delay(1);
        if(writeRegister(PWR_MGMNT_1, (char) 0x00) < 0){ // cycle 0, sleep 0, standby 0
            return -1;
        }
        if(writeRegister(PWR_MGMNT_2,DIS_GYRO) < 0){ // disable gyro measurements
            return -2;
        }
        if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_184) < 0){ // setting accel bandwidth to 184Hz
            return -3;
        }
        if(writeRegister(INT_ENABLE,INT_WOM_EN) < 0){ // enabling interrupt to wake on motion
            return -4;
        }
        if(writeRegister(MOT_DETECT_CTRL, (char) (ACCEL_INTEL_EN | ACCEL_INTEL_MODE)) < 0){ // enabling accel hardware intelligence
            return -5;
        }
        _womThreshold = (char) map((long) womThresh_mg, 0, 1020, 0, 255);
        if(writeRegister(WOM_THR,_womThreshold) < 0){ // setting wake on motion threshold
            return -6;
        }
        if(writeRegister(LP_ACCEL_ODR,(char) odr.getValue()) < 0){ // set frequency of wakeup
            return -7;
        }
        if(writeRegister(PWR_MGMNT_1,PWR_CYCLE) < 0){ // switch to accel low power mode
            return -8;
        }
        return 1;
    }

    public int readSensor() {
        // grab the data from the MPU9250
        if (readRegisters(ACCEL_OUT, (char) 21) < 0) {
            return -1;
        }
        // combine into 16 bit values
        _axcounts = (((int)_buffer[0]) << 8) | _buffer[1];
        _aycounts = (((int)_buffer[2]) << 8) | _buffer[3];
        _azcounts = (((int)_buffer[4]) << 8) | _buffer[5];
        _tcounts = (((int)_buffer[6]) << 8) | _buffer[7];
        _gxcounts = (((int)_buffer[8]) << 8) | _buffer[9];
        _gycounts = (((int)_buffer[10]) << 8) | _buffer[11];
        _gzcounts = (((int)_buffer[12]) << 8) | _buffer[13];
        _hxcounts = (((int)_buffer[15]) << 8) | _buffer[14];
        _hycounts = (((int)_buffer[17]) << 8) | _buffer[16];
        _hzcounts = (((int)_buffer[19]) << 8) | _buffer[18];
        // transform and convert to float values
        _ax = (((float)(tX[0]*_axcounts + tX[1]*_aycounts + tX[2]*_azcounts) * _accelScale) - _axb)*_axs;
        _ay = (((float)(tY[0]*_axcounts + tY[1]*_aycounts + tY[2]*_azcounts) * _accelScale) - _ayb)*_ays;
        _az = (((float)(tZ[0]*_axcounts + tZ[1]*_aycounts + tZ[2]*_azcounts) * _accelScale) - _azb)*_azs;
        _gx = ((float)(tX[0]*_gxcounts + tX[1]*_gycounts + tX[2]*_gzcounts) * _gyroScale) - _gxb;
        _gy = ((float)(tY[0]*_gxcounts + tY[1]*_gycounts + tY[2]*_gzcounts) * _gyroScale) - _gyb;
        _gz = ((float)(tZ[0]*_gxcounts + tZ[1]*_gycounts + tZ[2]*_gzcounts) * _gyroScale) - _gzb;
        _hx = (((float)(_hxcounts) * _magScaleX) - _hxb)*_hxs;
        _hy = (((float)(_hycounts) * _magScaleY) - _hyb)*_hys;
        _hz = (((float)(_hzcounts) * _magScaleZ) - _hzb)*_hzs;
        _t = ((((float) _tcounts) - _tempOffset)/_tempScale) + _tempOffset;
        return 1;
    }

    public float getAccelX_mss() {
        return _ax;
    }

    public float getAccelY_mss() {
        return _ay;
    }

    public float getAccelZ_mss() {
        return _az;
    }

    public float getGyroX_rads() {
        return _gx;
    }

    public float getGyroY_rads() {
        return _gy;
    }

    public float getGyroZ_rads() {
        return _gz;
    }

    public float getMagX_uT() {
        return _hx;
    }

    public float getMagY_uT() {
        return _hy;
    }

    public float getMagZ_uT() {
        return _hz;
    }

    public float getTemperature_C() {
        return _t;
    }

    public int calibrateGyro() {
        // set the range, bandwidth, and srd
        if (setGyroRange(GyroRange.GYRO_RANGE_250DPS) < 0) {
            return -1;
        }
        if (setDlpfBandwidth(DlpfBandwidth.DLPF_BANDWIDTH_20HZ) < 0) {
            return -2;
        }
        if (setSrd((char) 19) < 0) {
            return -3;
        }

        // take samples and find bias
        _gxbD = 0;
        _gybD = 0;
        _gzbD = 0;
        for (int i=0; i < _numSamples; i++) {
            readSensor();
            _gxbD += (getGyroX_rads() + _gxb)/((double)_numSamples);
            _gybD += (getGyroY_rads() + _gyb)/((double)_numSamples);
            _gzbD += (getGyroZ_rads() + _gzb)/((double)_numSamples);
            delay(20);
        }
        _gxb = (float)_gxbD;
        _gyb = (float)_gybD;
        _gzb = (float)_gzbD;

        // set the range, bandwidth, and srd back to what they were
        if (setGyroRange(_gyroRange) < 0) {
            return -4;
        }
        if (setDlpfBandwidth(_bandwidth) < 0) {
            return -5;
        }
        if (setSrd(_srd) < 0) {
            return -6;
        }
        return 1;
    }

    public float getGyroBiasX_rads() {
        return _gxb;
    }

    public float getGyroBiasY_rads() {
        return _gyb;
    }

    public float getGyroBiasZ_rads() {
        return _gzb;
    }

    public void setGyroBiasX_rads(float bias) {
        _gxb = bias;
    }

    public void setGyroBiasY_rads(float bias) {
        _gyb = bias;
    }

    public void setGyroBiasZ_rads(float bias) {
        _gzb = bias;
    }

    public int calibrateAccel() {
// set the range, bandwidth, and srd
        if (setAccelRange(AccelRange.ACCEL_RANGE_2G) < 0) {
            return -1;
        }
        if (setDlpfBandwidth(DlpfBandwidth.DLPF_BANDWIDTH_20HZ) < 0) {
            return -2;
        }
        if (setSrd((char) 19) < 0) {
            return -3;
        }

        // take samples and find min / max
        _axbD = 0;
        _aybD = 0;
        _azbD = 0;
        for (int i=0; i < _numSamples; i++) {
            readSensor();
            _axbD += (getAccelX_mss()/_axs + _axb)/((double)_numSamples);
            _aybD += (getAccelY_mss()/_ays + _ayb)/((double)_numSamples);
            _azbD += (getAccelZ_mss()/_azs + _azb)/((double)_numSamples);
            delay(20);
        }
        if (_axbD > 9.0f) {
            _axmax = (float)_axbD;
        }
        if (_aybD > 9.0f) {
            _aymax = (float)_aybD;
        }
        if (_azbD > 9.0f) {
            _azmax = (float)_azbD;
        }
        if (_axbD < -9.0f) {
            _axmin = (float)_axbD;
        }
        if (_aybD < -9.0f) {
            _aymin = (float)_aybD;
        }
        if (_azbD < -9.0f) {
            _azmin = (float)_azbD;
        }

        // find bias and scale factor
        if ((Math.abs(_axmin) > 9.0f) && (Math.abs(_axmax) > 9.0f)) {
            _axb = (_axmin + _axmax) / 2.0f;
            _axs = G/((Math.abs(_axmin) + Math.abs(_axmax)) / 2.0f);
        }
        if ((Math.abs(_aymin) > 9.0f) && (Math.abs(_aymax) > 9.0f)) {
            _ayb = (_aymin + _aymax) / 2.0f;
            _ays = G/((Math.abs(_aymin) + Math.abs(_aymax)) / 2.0f);
        }
        if ((Math.abs(_azmin) > 9.0f) && (Math.abs(_azmax) > 9.0f)) {
            _azb = (_azmin + _azmax) / 2.0f;
            _azs = G/((Math.abs(_azmin) + Math.abs(_azmax)) / 2.0f);
        }

        // set the range, bandwidth, and srd back to what they were
        if (setAccelRange(_accelRange) < 0) {
            return -4;
        }
        if (setDlpfBandwidth(_bandwidth) < 0) {
            return -5;
        }
        if (setSrd(_srd) < 0) {
            return -6;
        }
        return 1;
    }

    public float getAccelBiasX_mss() {
        return _axb;
    }

    public float getAccelScaleFactorX() {
        return _axs;
    }

    public float getAccelBiasY_mss() {
        return _ayb;
    }

    public float getAccelScaleFactorY() {
        return _ays;
    }

    public float getAccelBiasZ_mss() {
        return _azb;
    }

    public float getAccelScaleFactorZ() {
        return _azs;
    }

    public void setAccelCalX(float bias, float scaleFactor) {
        _axb = bias;
        _axs = scaleFactor;
    }

    public void setAccelCalY(float bias, float scaleFactor) {
        _ayb = bias;
        _ays = scaleFactor;
    }

    public void setAccelCalZ(float bias, float scaleFactor) {
        _azb = bias;
        _azs = scaleFactor;
    }

    public int calibrateMag() {
        // set the srd
        if (setSrd((char) 19) < 0) {
            return -1;
        }

        // get a starting set of data
        readSensor();
        _hxmax = getMagX_uT();
        _hxmin = getMagX_uT();
        _hymax = getMagY_uT();
        _hymin = getMagY_uT();
        _hzmax = getMagZ_uT();
        _hzmin = getMagZ_uT();

        // collect data to find max / min in each channel
        _counter = 0;
        while (_counter < _maxCounts) {
            _delta = 0.0f;
            _framedelta = 0.0f;
            readSensor();
            _hxfilt = (_hxfilt*((float)_coeff-1)+(getMagX_uT()/_hxs+_hxb))/((float)_coeff);
            _hyfilt = (_hyfilt*((float)_coeff-1)+(getMagY_uT()/_hys+_hyb))/((float)_coeff);
            _hzfilt = (_hzfilt*((float)_coeff-1)+(getMagZ_uT()/_hzs+_hzb))/((float)_coeff);
            if (_hxfilt > _hxmax) {
                _delta = _hxfilt - _hxmax;
                _hxmax = _hxfilt;
            }
            if (_delta > _framedelta) {
                _framedelta = _delta;
            }
            if (_hyfilt > _hymax) {
                _delta = _hyfilt - _hymax;
                _hymax = _hyfilt;
            }
            if (_delta > _framedelta) {
                _framedelta = _delta;
            }
            if (_hzfilt > _hzmax) {
                _delta = _hzfilt - _hzmax;
                _hzmax = _hzfilt;
            }
            if (_delta > _framedelta) {
                _framedelta = _delta;
            }
            if (_hxfilt < _hxmin) {
                _delta = Math.abs(_hxfilt - _hxmin);
                _hxmin = _hxfilt;
            }
            if (_delta > _framedelta) {
                _framedelta = _delta;
            }
            if (_hyfilt < _hymin) {
                _delta = Math.abs(_hyfilt - _hymin);
                _hymin = _hyfilt;
            }
            if (_delta > _framedelta) {
                _framedelta = _delta;
            }
            if (_hzfilt < _hzmin) {
                _delta = Math.abs(_hzfilt - _hzmin);
                _hzmin = _hzfilt;
            }
            if (_delta > _framedelta) {
                _framedelta = _delta;
            }
            if (_framedelta > _deltaThresh) {
                _counter = 0;
            } else {
                _counter++;
            }
            delay(20);
        }

        // find the magnetometer bias
        _hxb = (_hxmax + _hxmin) / 2.0f;
        _hyb = (_hymax + _hymin) / 2.0f;
        _hzb = (_hzmax + _hzmin) / 2.0f;

        // find the magnetometer scale factor
        _hxs = (_hxmax - _hxmin) / 2.0f;
        _hys = (_hymax - _hymin) / 2.0f;
        _hzs = (_hzmax - _hzmin) / 2.0f;
        _avgs = (_hxs + _hys + _hzs) / 3.0f;
        _hxs = _avgs/_hxs;
        _hys = _avgs/_hys;
        _hzs = _avgs/_hzs;

        // set the srd back to what it was
        if (setSrd(_srd) < 0) {
            return -2;
        }
        return 1;
    }

    public float getMagBiasX_uT() {
        return _hxb;
    }

    public float getMagScaleFactorX() {
        return _hxs;
    }

    public float getMagBiasY_uT() {
        return _hyb;
    }

    public float getMagScaleFactorY() {
        return _hys;
    }

    public float getMagBiasZ_uT() {
        return _hzb;
    }

    public float getMagScaleFactorZ() {
        return _hzs;
    }

    public void setMagCalX(float bias, float scaleFactor) {
        _hxb = bias;
        _hxs = scaleFactor;
    }

    public void setMagCalY(float bias, float scaleFactor) {
        _hyb = bias;
        _hys = scaleFactor;
    }

    public void setMagCalZ(float bias, float scaleFactor) {
        _hzb = bias;
        _hzs = scaleFactor;
    }

    private int writeRegister(char subAddress, char data) {
        deviceClient.engage();
        deviceClient.write8(subAddress, data);
        deviceClient.disengage();

        delay(10);

        /* read back the register */
        readRegisters(subAddress, (char) 1);
        /* check the read back register against the written register */
        if(_buffer[0] == data) {
            return 1;
        }
        else{
            return -1;
        }
    }

    private int readRegisters(char subAddress, char count) {
        deviceClient.engage();
        _buffer = deviceClient.read(subAddress, count);
        deviceClient.disengage();

        if (_buffer.length == count) {
            return 1;
        } else {
            return -1;
        }
    }

    private int writeAK8963Register(char subAddress, char data) {
        // set slave 0 to the AK8963 and set for write
        if (writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR) < 0) {
            return -1;
        }
        // set the register to the desired AK8963 sub address
        if (writeRegister(I2C_SLV0_REG,subAddress) < 0) {
            return -2;
        }
        // store the data for write
        if (writeRegister(I2C_SLV0_DO,data) < 0) {
            return -3;
        }
        // enable I2C and send 1 byte
        if (writeRegister(I2C_SLV0_CTRL, (char) (I2C_SLV0_EN | (char)1)) < 0) {
            return -4;
        }
        // read the register and confirm
        if (readAK8963Registers(subAddress, (char) 1) < 0) {
            return -5;
        }
        if(_buffer[0] == data) {
            return 1;
        } else{
            return -6;
        }
    }

    private int readAK8963Registers(char subAddress, char count) {
        // set slave 0 to the AK8963 and set for read
        if (writeRegister(I2C_SLV0_ADDR, (char) (AK8963_I2C_ADDR | I2C_READ_FLAG)) < 0) {
            return -1;
        }
        // set the register to the desired AK8963 sub address
        if (writeRegister(I2C_SLV0_REG,subAddress) < 0) {
            return -2;
        }
        // enable I2C and request the bytes
        if (writeRegister(I2C_SLV0_CTRL, (char) (I2C_SLV0_EN | count)) < 0) {
            return -3;
        }
        delay(1); // takes some time for these registers to fill
        // read the bytes off the MPU9250 EXT_SENS_DATA registers

        byte prev[] = _buffer.clone();

        _status = readRegisters(EXT_SENS_DATA_00,count);

        _buffer = prev;

        return _status;
    }

    private int whoAmI() {
        // read the WHO AM I register
        if (readRegisters(WHO_AM_I, (char) 1) < 0) {
            return -1;
        }
        // return the register value
        return _buffer[0];
    }

    private int whoAmIAK8963() {
        // read the WHO AM I register
        if (readAK8963Registers(AK8963_WHO_AM_I, (char) 1) < 0) {
            return -1;
        }
        // return the register value
        return _buffer[0];
    }

    private void delay(int milliseconds) {
        try {
            sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private long map(long x, long in_min, long in_max, long out_min, long out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize() {

        return true;
    }

    @Override
    public String getDeviceName() {
        return "MPU9250 Accelerometer, Gyro, and Compass Module";
    }

}
