//% blockId="MicrobitRobot" block="Toolbox for Dad's Robot"
//% color="#274272" weight=20 icon="\uf1ae"
namespace MicrobitRobot {
    const PCA9685_OSC_FREQENCE = 25000000;
    const PCA9685_RESTART_DELAY = 500;
    const PCA9685_BASE_ADDR = 0x40;
    const MODE_1_SUB_ADDR = 0x00;
    const PRESCARE_SUB_ADDR = 0xfe;
    const PWM_STEP_MIN = 800;
    const PWM_STEP_MAX = 4096;
    const PWM_UPDATE_RATE = 50;
    const MOTOR_DELAY_TIME = 500000;
    const LED_0_SUB_ADDR = 0x06;
    const LED_SUB_ADDR_OFFSET = 4;

    let _initialized = false;
    let _dir_lamp_flash = false;

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

    //% blockId="positionServoMotor" block="position %motor| at %angle degrees."
    //% color="#cc0000"
    //% angle.min=-90 angle.max=90 angle.default=0
    export function positionServoMotor() {
        if (!_initialized) initPCA9685();

        /*
        let opMotor = 0;
        switch (motor) {
            case ServoMotorId.J2:
                opMotor = SERVO_MOTOR_1_CHANNEL;
                break;
            case ServoMotorId.J3:
                opMotor = SERVO_MOTOR_2_CHANNEL;
                break;
            case ServoMotorId.J4:
                opMotor = SERVO_MOTOR_3_CHANNEL;
                break;
        }

        let opAngle: number = angle + SERVO_MOTOR_ROTATION_DEGREE;
        let opDuty: number =
            opAngle * SERVO_MOTOR_DUTY_PER_DEGREE + SERVO_MOTOR_MIN_DUTY;
        let opTravel: number = (opDuty / SERVO_MOTOR_ONE_CYCLE) * PWM_STEP_MAX;

        let buffs = pins.createBuffer(5);
        buffs[0] = LED_0_SUB_ADDR + LED_SUB_ADDR_OFFSET * opMotor;
        buffs[1] = 0;
        buffs[2] = 0;
        buffs[3] = 0;
        buffs[4] = 0;
        pins.i2cWriteBuffer(PCA9685_BASE_ADDR, buffs);

        control.waitMicros(15);
        buffs[0] = LED_0_SUB_ADDR + LED_SUB_ADDR_OFFSET * opMotor;
        buffs[1] = 0;
        buffs[2] = 0;
        buffs[3] = opTravel & 0xff;
        buffs[4] = (opTravel >> 8) & 0x0f;
        pins.i2cWriteBuffer(PCA9685_BASE_ADDR, buffs);
        */
    }
}
