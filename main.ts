//% blockId="MicrobitRobot" block="Toolbox for Dad's Robot"
//% color="#274272" weight=20 icon="\uf1ae"
namespace MicrobitRobot {
    const PCA9685_OSC_FREQENCE = 25000000;
    const PCA9685_RESTART_DELAY = 500;
    const PCA9685_BASE_ADDR = 0x40;
    const MODE_1_SUB_ADDR = 0x00;
    const PRESCARE_SUB_ADDR = 0xfe;
    const PWM_STEP_MAX = 4096;
    const PWM_UPDATE_RATE = 50;
    const MOTOR_DELAY_TIME = 500000;
    const SERVO_0_SUB_ADDR = 0x26;
    const SERVO_SUB_ADDR_OFFSET = 4;
    const SERVO_MOTOR_ROTATION_DEGREE = 90;
    const SERVO_MOTOR_MIN_DUTY = 600; // us
    const SERVO_MOTOR_MAX_DUTY = 2400; // us
    const SERVO_MOTOR_ONE_CYCLE = 20000; // us
    const SERVO_MOTOR_DUTY_PER_DEGREE =
        (SERVO_MOTOR_MAX_DUTY - SERVO_MOTOR_MIN_DUTY) /
        (2 * SERVO_MOTOR_ROTATION_DEGREE);

    let _initialized = false;
    let _robotBody = [-1, -1, -1, -1, -1, -1, -1, -1];

    function setPwmUpdateRate(rate: number): void {
        let prescare = PCA9685_OSC_FREQENCE / PWM_STEP_MAX / rate - 1;

        pins.i2cWriteNumber(
            PCA9685_BASE_ADDR,
            MODE_1_SUB_ADDR,
            NumberFormat.Int8BE
        );
        let older = pins.i2cReadNumber(PCA9685_BASE_ADDR, NumberFormat.UInt8BE);
        let newer = (older & 0x7f) | 0x10;

        let buffs = pins.createBuffer(2);
        buffs[0] = MODE_1_SUB_ADDR;
        buffs[1] = newer;
        pins.i2cWriteBuffer(PCA9685_BASE_ADDR, buffs);

        buffs[0] = PRESCARE_SUB_ADDR;
        buffs[1] = prescare;
        pins.i2cWriteBuffer(PCA9685_BASE_ADDR, buffs);

        control.waitMicros(PCA9685_RESTART_DELAY);
        buffs[0] = MODE_1_SUB_ADDR;
        buffs[1] = older | 0xa1;
        pins.i2cWriteBuffer(PCA9685_BASE_ADDR, buffs);
    }

    function initPCA9685(): void {
        let buffs = pins.createBuffer(2);
        buffs[0] = MODE_1_SUB_ADDR;
        buffs[1] = 0x0;
        pins.i2cWriteBuffer(PCA9685_BASE_ADDR, buffs);
        setPwmUpdateRate(PWM_UPDATE_RATE);
        _initialized = true;
    }

    function calcServoAngle(angle: number): number {
        let opAngle: number = angle + SERVO_MOTOR_ROTATION_DEGREE;
        let opDuty: number =
            opAngle * SERVO_MOTOR_DUTY_PER_DEGREE + SERVO_MOTOR_MIN_DUTY;
        let opTravel: number = (opDuty / SERVO_MOTOR_ONE_CYCLE) * PWM_STEP_MAX;

        return opTravel;
    }

    export enum ServoMotorPin {
        //% blockId="ServoMotorS1" block="S1"
        S1 = 0,
        //% blockId="ServoMotorS2" block="S2"
        S2,
        //% blobkId="ServoMotorS3" block="S3"
        S3,
        //% blockId="ServoMotorS4" block="S4"
        S4,
        //% blockId="ServoMotorS5" block="S5"
        S5,
        //% blockId="ServoMotorS6" block="S6"
        S6,
        //% blockId="ServoMotorS7" block="S7"
        S7,
        //% blockId="ServoMotorS8" block="S8"
        S8
    }

    export enum RobotBody {
        //% blobkId="RobotRightArm" block="RIGHT ARM"
        RIGHT_ARM = 0,
        //% blockId="RobotRightHand" block="RIGHT HAND"
        RIGHT_HAND,
        //% blockId="RobotRightLeg" block="RIGHT LEG"
        RIGHT_LEG,
        //% blockId="RobotRightFoot" block="RIGHT FOOT"
        RIGHT_FOOT,
        //% blockId="RobotLeftArm" block="LEFT ARM"
        LEFT_ARM,
        //% blockId="RobotLeftHand" block="LEFT HAND"
        LEFT_HAND,
        //% blockId="RobotLeftLeg" block="LEFT LEG"
        LEFT_LEG,
        //% blockId="RobotLeftFoot" block="LEFT FOOT"
        LEFT_FOOT
    }

    function getServoMotorId(motor: ServoMotorPin): number {
        switch (motor) {
            case ServoMotorPin.S1:
                return 0;
            case ServoMotorPin.S2:
                return 1;
            case ServoMotorPin.S3:
                return 2;
            case ServoMotorPin.S4:
                return 3;
            case ServoMotorPin.S5:
                return 4;
            case ServoMotorPin.S6:
                return 5;
            case ServoMotorPin.S7:
                return 6;
            case ServoMotorPin.S8:
                return 7;
            default:
                return -1;
        }
    }

    //% blobkId="mapRobotServo" block="set servo motor %motor| to %body"
    //% color="#cc0000"
    export function mapRobotServo(body: RobotBody, motor: ServoMotorPin) {
        switch (body) {
            case RobotBody.RIGHT_ARM:
                _robotBody[0] = getServoMotorId(motor);
                break;
            case RobotBody.RIGHT_HAND:
                _robotBody[1] = getServoMotorId(motor);
                break;
            case RobotBody.RIGHT_LEG:
                _robotBody[2] = getServoMotorId(motor);
                break;
            case RobotBody.RIGHT_FOOT:
                _robotBody[3] = getServoMotorId(motor);
                break;
            case RobotBody.LEFT_ARM:
                _robotBody[4] = getServoMotorId(motor);
                break;
            case RobotBody.LEFT_HAND:
                _robotBody[5] = getServoMotorId(motor);
                break;
            case RobotBody.LEFT_LEG:
                _robotBody[6] = getServoMotorId(motor);
                break;
            case RobotBody.LEFT_FOOT:
                _robotBody[7] = getServoMotorId(motor);
                break;
        }
    }

    //% blockId="positionServoMotor" block="position %motor| at %angle degrees."
    //% color="#cc0000"
    //% angle.min=-90 angle.max=90 angle.default=0
    export function positionServoMotor(body: RobotBody, angle: number) {
        if (!_initialized) initPCA9685();

        let opTravel = calcServoAngle(angle);
        let opMotor = _robotBody[body];
        let buffs = pins.createBuffer(5);

        buffs[0] = SERVO_0_SUB_ADDR + SERVO_SUB_ADDR_OFFSET * opMotor;
        buffs[1] = 0;
        buffs[2] = 0;
        buffs[3] = 0;
        buffs[4] = 0;
        pins.i2cWriteBuffer(PCA9685_BASE_ADDR, buffs);

        control.waitMicros(15);
        buffs[0] = SERVO_0_SUB_ADDR + SERVO_SUB_ADDR_OFFSET * opMotor;
        buffs[1] = 0;
        buffs[2] = 0;
        buffs[3] = opTravel & 0xff;
        buffs[4] = (opTravel >> 8) & 0x0f;
        pins.i2cWriteBuffer(PCA9685_BASE_ADDR, buffs);
    }
}
