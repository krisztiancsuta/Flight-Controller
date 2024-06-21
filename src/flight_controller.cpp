#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"
// #include "MPU6050_6Axis_MotionApps_V6_12.h"
// #include "helper_3dmath.h"
#include "pindef.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include <math.h>

uint8_t CONFIG = 0x1A;
uint8_t PWR_MGMT_1 = 0x6B;
uint8_t ACCEL_CONFIG = 0x1C;
uint8_t GYRO_CONFIG = 0x1B;
uint8_t MPU6050_ADDRESS = 0x68;
uint8_t WHO_AM_I = 0x75;
uint8_t id = 0x00;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.4;  // Gain setting for the roll P-controller (1.3)
float pid_i_gain_roll = 0.05; // Gain setting for the roll I-controller (0.05)
float pid_d_gain_roll = 15;   // Gain setting for the roll D-controller (15)
int pid_max_roll = 400;       // Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll; // Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll; // Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll; // Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;         // Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;  // Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02; // Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;  // Gain setting for the pitch D-controller.
int pid_max_yaw = 400;       // Maximum output of the PID-controller (+/-)

bool auto_level = true; // Set to false for manual flight. Set to true for autolevel flight
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t last_channel_1, last_channel_2, last_channel_3, last_channel_4;
uint8_t highByte, lowByte;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
int16_t cal_int, start, gyro_address;
int receiver_input[5];
uint16_t temperature;
int16_t acc_axis[4], gyro_axis[4];
float roll_level_adjust, pitch_level_adjust;

int32_t acc_x, acc_y, acc_z, acc_total_vector;
uint32_t timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
uint32_t timer_1, timer_2, timer_3, timer_4, current_time;
uint32_t loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[4];
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
bool gyro_angles_set;

uint32_t start_i2c(uint8_t mpu_scl_pin, uint8_t mpu_sda_pin, uint32_t i2c_clk)
{
    uint32_t clk = i2c_init(i2c0, i2c_clk);
    gpio_set_function(mpu_sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(mpu_scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(mpu_sda_pin);
    gpio_pull_up(mpu_scl_pin);
    return clk;
}

void initLED()
{
    if (cyw43_arch_init())
    {
        exit(3);
    }
}

void blinkLED(uint16_t interval)
{
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(interval);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(interval);
}
/*
bool i2c_write_reg(i2c_inst_t* i2c, uint8_t addr, uint8_t reg_addr, uint8_t const* reg_data, size_t length)
{
    uint8_t buf[length + 1];
    buf[0] = reg_addr;
    __builtin_memcpy(&buf[1], reg_data, length);
    return i2c_write_timeout_us(i2c, addr, buf, length + 1, false, 10000) == (length + 1);
}

*/

bool write_register(i2c_inst *i2c, uint8_t sensor_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t len)
{
    uint8_t buffer[len + 1];
    buffer[0] = reg_addr;
    __builtin_memcpy(&buffer[1], reg_data, len);
    return i2c_write_blocking(i2c, sensor_addr, buffer, len + 1, false) == (len + 1);
}

void set_gyro_registers()
{

    i2c_write_blocking(i2c0, 0x68, &WHO_AM_I, 1, true);
    i2c_read_blocking(i2c0, 0x68, &id, 1, false);
    if (id != 0x68)
    {
        printf("MPU6050 not found\n");
        return;
    }
    else
        printf("MPU6050 ID: 0x%x\n", id);

    // Wake up the MPU-6050 since it starts in sleep mode
    uint8_t reg_data = 0x00;
    write_register(i2c0, MPU6050_ADDRESS, PWR_MGMT_1, &reg_data, 1);
    // Gyro config
    reg_data = 0x08;
    write_register(i2c0, MPU6050_ADDRESS, GYRO_CONFIG, &reg_data, 1);
    // Accel config
    reg_data = 0x10;
    write_register(i2c0, MPU6050_ADDRESS, ACCEL_CONFIG, &reg_data, 1);
    // Config set Digital Low Pass Filter to ~43Hz)
    reg_data = 0x03;
    write_register(i2c0, MPU6050_ADDRESS, CONFIG, &reg_data, 1);
}

void read_gyro_data()
{
    uint8_t buf[14];

    uint8_t reg = 0x3B; // Starting with register 0x3B (ACCEL_XOUT_H)
    i2c_write_blocking(i2c0, 0x68, &reg, 1, true);
    i2c_read_blocking(i2c0, 0x68, buf, sizeof(buf), false);
    acc_axis[0] = (buf[0] << 8) | buf[1];
    acc_axis[1] = (buf[2] << 8) | buf[3];
    acc_axis[2] = (buf[4] << 8) | buf[5];
    temperature = (((buf[6] << 8) | buf[7]) + 12421) / 340;
    gyro_axis[0] = (buf[8] << 8) | buf[9];
    gyro_axis[1] = (buf[10] << 8) | buf[11];
    gyro_axis[2] = (buf[12] << 8) | buf[13];

    if (cal_int == 2000)
    {
        gyro_axis[0] -= gyro_axis_cal[0];
        gyro_axis[1] -= gyro_axis_cal[1];
        gyro_axis[2] -= gyro_axis_cal[2];
    }

    gyro_roll = gyro_axis[0];
    gyro_pitch = gyro_axis[1];
    gyro_yaw = gyro_axis[2];

    acc_x = acc_axis[0] - 86;
    acc_y = acc_axis[1] - 8;
    acc_z = acc_axis[2] + 216;
}

void PWM_init(uint8_t pin, uint8_t pin2, uint8_t pin3, uint8_t pin4)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    gpio_set_function(pin2, GPIO_FUNC_PWM);
    gpio_set_function(pin3, GPIO_FUNC_PWM);
    gpio_set_function(pin4, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    uint slice_num_2 = pwm_gpio_to_slice_num(pin3);
    pwm_set_clkdiv(slice_num, 125.0f);
    pwm_set_wrap(slice_num, 4000);
    pwm_set_clkdiv(slice_num_2, 125.0f);
    pwm_set_wrap(slice_num_2, 4000);
    pwm_set_enabled(slice_num, true);
    pwm_set_enabled(slice_num_2, true);
}

void set_speed(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
    uint8_t slice_num = pwm_gpio_to_slice_num(ESC1_PIN);
    uint8_t slice_num_2 = pwm_gpio_to_slice_num(ESC3_PIN);
    if (1000 <= m1 && m1 <= 2000 && 1000 <= m2 && m2 <= 2000 && 1000 <= m3 && m3 <= 2000 && 1000 <= m4 && m4 <= 2000)
    {
        pwm_set_chan_level(slice_num, PWM_CHAN_A, m1);
        pwm_set_chan_level(slice_num, PWM_CHAN_B, m2);
        pwm_set_chan_level(slice_num_2, PWM_CHAN_A, m3);
        pwm_set_chan_level(slice_num_2, PWM_CHAN_B, m4);
    }
    else
    {
        pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
        pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
        pwm_set_chan_level(slice_num_2, PWM_CHAN_A, 0);
        pwm_set_chan_level(slice_num_2, PWM_CHAN_B, 0);
    }
    return;
}

void set_up_adc(uint8_t adc_pin, uint8_t adc_channel)
{
    adc_init();
    adc_gpio_init(adc_pin);
    adc_select_input(adc_channel);
}

uint16_t read_battery_voltage()
{
    // Max voltage is 12.6V of a 3S Lipo battery
    // Ref voltage is 3.3V of the Pico
    // We need to divide tho voltage with a voltage divider
    //  Vo = Vs * (R1 + R2) / R2
    // voltage divider R1:R2 = 3:1
    // Vs = 13.0V (max) -> Vo = 3.3V (max) ADC input will be in safe range
    constexpr unsigned R1 = 30000; // R Ohm
    constexpr unsigned R2 = 10000; // R Ohm
    constexpr float devided_conversion = 3.3f / (1 << 12);
    constexpr float original_conversion = devided_conversion * (R1 + R2) / R2;
    // Accuracy is 3.3V / 4096 = 0.0008056640625V = 0.8056640625mV
    // Accuracy of the voltage divider is 0.8056640625mV * 3 = 2.4179921875mV

    // ADC_RANGE 12 bit value = 0-4095 = 0-3.3V
    return (uint16_t)(adc_read() * original_conversion * 100);
    // return beetwen 0-1300 aprox
}
void setup()
{
    start_i2c(MPU_SCL_PIN, MPU_SDA_PIN, 400000);

    initLED();
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1); // turn on warning led
    sleep_ms(1000);
    set_gyro_registers();

    PWM_init(ESC1_PIN, ESC2_PIN, ESC3_PIN, ESC4_PIN);
    set_speed(1000, 1000, 1000, 1000);

    set_up_adc(ADC_PIN, ADC_CHANNEL); // init adc pin at channel 0

    // Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
    for (cal_int = 0; cal_int < 2000; cal_int++)
    {
        if (cal_int % 15 == 0)
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0); // turn off warning led
        else
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1); // turn on warning led

        // Read the gyro output.
        read_gyro_data();
        gyro_axis_cal[0] += gyro_axis[0]; // Ad roll value to gyro_roll_cal.
        gyro_axis_cal[1] += gyro_axis[1]; // Ad pitch value to gyro_pitch_cal.
        gyro_axis_cal[2] += gyro_axis[2]; // Ad yaw value to gyro_yaw_cal.
        sleep_ms(3);                      // Wait 3 milliseconds before the next loop.
    }

    // Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.

    gyro_axis_cal[0] /= 2000; // Divide the roll total by 2000.
    gyro_axis_cal[1] /= 2000; // Divide the pitch total by 2000.
    gyro_axis_cal[2] /= 2000; // Divide the yaw total by 2000.

    // TODO: Reciever

    // TODO: Battery voltage
    battery_voltage = read_battery_voltage();

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0); // turn off warning led
}

void calculate_pid()
{
    // Roll calculations
    pid_error_temp = gyro_roll_input - pid_roll_setpoint;
    pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
    if (pid_i_mem_roll > pid_max_roll)
        pid_i_mem_roll = pid_max_roll;
    else if (pid_i_mem_roll < pid_max_roll * -1)
        pid_i_mem_roll = pid_max_roll * -1;

    pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
    if (pid_output_roll > pid_max_roll)
        pid_output_roll = pid_max_roll;
    else if (pid_output_roll < pid_max_roll * -1)
        pid_output_roll = pid_max_roll * -1;

    pid_last_roll_d_error = pid_error_temp;

    // Pitch calculations
    pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
    pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
    if (pid_i_mem_pitch > pid_max_pitch)
        pid_i_mem_pitch = pid_max_pitch;
    else if (pid_i_mem_pitch < pid_max_pitch * -1)
        pid_i_mem_pitch = pid_max_pitch * -1;

    pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
    if (pid_output_pitch > pid_max_pitch)
        pid_output_pitch = pid_max_pitch;
    else if (pid_output_pitch < pid_max_pitch * -1)
        pid_output_pitch = pid_max_pitch * -1;

    pid_last_pitch_d_error = pid_error_temp;

    // Yaw calculations
    pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
    pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
    if (pid_i_mem_yaw > pid_max_yaw)
        pid_i_mem_yaw = pid_max_yaw;
    else if (pid_i_mem_yaw < pid_max_yaw * -1)
        pid_i_mem_yaw = pid_max_yaw * -1;

    pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
    if (pid_output_yaw > pid_max_yaw)
        pid_output_yaw = pid_max_yaw;
    else if (pid_output_yaw < pid_max_yaw * -1)
        pid_output_yaw = pid_max_yaw * -1;

    pid_last_yaw_d_error = pid_error_temp;
}
int main()
{

    stdio_init_all();
    sleep_ms(1000);
    setup();

    while (true)
    {
        gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);    // Gyro pid input is deg/sec.
        gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3); // Gyro pid input is deg/sec.
        gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);       // Gyro pid input is deg/sec.

        // Gyro angle calculations
        // 0.0000611 = 1 / (250Hz / 65.5)
        angle_pitch += gyro_pitch * 0.0000611; // Calculate the traveled pitch angle and add this to the angle_pitch variable.
        angle_roll += gyro_roll * 0.0000611;

        // 0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
        angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066); // If the IMU has yawed transfer the roll angle to the pitch angel.
        angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066); // If the IMU has yawed transfer the pitch angle to the roll angel.

        // Accelerometer angle calculations

        acc_total_vector = sqrt(((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)));

        if (abs(acc_y) < acc_total_vector)
        {                                                                     // Prevent the asin function to produce a NaN
            angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296; // Calculate the pitch angle.
        }
        if (abs(acc_x) < acc_total_vector)
        {                                                                     // Prevent the asin function to produce a NaN
            angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296; // Calculate the roll angle.
        }
        angle_pitch_acc -= 1.6033545646; // Accelerometer calibration value for pitch.
        angle_roll_acc -= 0.5455334096;  // Accelerometer calibration value for roll.

        angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004; // Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
        angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;

        pitch_level_adjust = angle_pitch * 15; // Calculate the pitch angle correction
        roll_level_adjust = angle_roll * 15;   // Calculate the roll angle correction

        if (!auto_level)
        {                           // If the quadcopter is not in auto-level mode
            pitch_level_adjust = 0; // Set the pitch angle correction to zero.
            roll_level_adjust = 0;  // Set the roll angle correcion to zero.
        }

        // TODO: Reciever

        start = 1;
        if (start == 1)
        {
            start = 2;
            angle_pitch = angle_pitch_acc; // Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
            angle_roll = angle_roll_acc;   // Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
            gyro_angles_set = true;        // Set the IMU started flag.

            // Reset the PID controllers for a bumpless start.
            pid_i_mem_roll = 0;
            pid_last_roll_d_error = 0;
            pid_i_mem_pitch = 0;
            pid_last_pitch_d_error = 0;
            pid_i_mem_yaw = 0;
            pid_last_yaw_d_error = 0;
        }

        pid_roll_setpoint = 0;
        pid_pitch_setpoint = 0;
        pid_yaw_setpoint = 0;

        calculate_pid();

        battery_voltage = battery_voltage * 0.92 + read_battery_voltage() * 0.09853;

        if (battery_voltage < 1000 && battery_voltage > 600)
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1); // turn on warning led if battery voltage is low

        throttle = 1400;
        // ESC Speed calculation
        esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; // Calculate the pulse for esc 1 (front-right - CCW)
        esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; // Calculate the pulse for esc 2 (rear-right - CW)
        esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; // Calculate the pulse for esc 3 (rear-left - CCW)
        esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; // Calculate the pulse for esc 4 (front-left - CW)

        // TODO: Battery voltage drop compensation
        sleep_us(3918);

        read_gyro_data();

        set_speed(esc_1, esc_2, esc_3, esc_4);

        printf("ra:%3f pa:%3f\n", angle_roll_acc, angle_pitch_acc);
        printf("1| %d  2| %d   3| %d   4| %d  \n", esc_1, esc_2, esc_3, esc_4);
    }

    return 0;
}