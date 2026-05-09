#include <stdio.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_netif.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rcl/init_options.h>
#include <rcl/node.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>

#include "uros_network_interfaces.h"

extern const char server_cert_pem_start[] asm("_binary_server_crt_start");
extern const char server_cert_pem_end[]   asm("_binary_server_crt_end");

static const char *TAG = "hw_test_node";

/* =====================
 * OTA button
 * ===================== */
#define OTA_BUTTON_GPIO GPIO_NUM_0
static volatile bool ota_requested = false;
static volatile bool ota_in_progress = false;

/* =====================
 * OTA dynamic URL
 * ===================== */
char dynamic_ota_url[192] = "";

/* =====================
 * L298N motor pins
 * ===================== */
#define ENA 25
#define IN1 26
#define IN2 27

#define ENB 14
#define IN3 32
#define IN4 33

/* =====================
 * MPU6050 I2C setup
 * ===================== */
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_SDA_IO       21
#define I2C_MASTER_SCL_IO       22
#define I2C_MASTER_FREQ_HZ      100000

#define MPU6050_ADDR            0x68
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_ACCEL_XOUT_H    0x3B

#define ACCEL_SCALE             16384.0f   // ±2g
#define GYRO_SCALE              131.0f     // ±250 deg/s

#define MPU6050_WHO_AM_I        0x75
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_GYRO_CONFIG     0x1B

/* =====================
 * LEDC setup
 * ===================== */
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_HIGH_SPEED_MODE
#define LEDC_OUTPUT_IO_A        ENA
#define LEDC_OUTPUT_IO_B        ENB
#define LEDC_CHANNEL_A          LEDC_CHANNEL_0
#define LEDC_CHANNEL_B          LEDC_CHANNEL_1
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY          5000

/* =====================
 * Control parameters
 * ===================== */

#define IMU_LOG_PERIOD_MS       50   // 20Hz logging
/*
 * Split path errors 부호가 반대로 동작하면
 * 1.0f 를 -1.0f 로 바꾸면 됨.
 */
#define STEER_SIGN              1.0f

/*
 * 모터 최소 구동 PWM.
 * 0이면 정지, 1~MIN_DRIVE_PWM-1이면 MIN_DRIVE_PWM으로 보정.
 */
#define MIN_DRIVE_PWM           140

//정지 마찰을 깨기 위한 짧은 고출력 kick PWM.
#define START_KICK_PWM          185
#define START_KICK_TIME_MS      100

//ArUco marker가 하나만 보일 때 화면 안으로 복구하기 위한 제자리 회전 PWM.
#define SEARCH_TURN_PWM         160
#define SEARCH_CMD_TIMEOUT_MS   500

// Lost-recovery uses the last valid steering-equivalent error.
// Normal control uses only split path errors below.
float Kp = 1.05f;
float Ki = 0.0f;
float Kd = 0.0f;

// Split path-error gains.
// steer = Ky*lateral_px + Ktheta*heading_deg + Klook*lookahead_deg
// Large heading error is blended smoothly toward a stronger heading-first correction.
float Ky = 0.035f;
float Ktheta = 0.85f;
float Klook = 0.55f;
float Ktheta_high = 2.20f;

#define HEADING_PRIORITY_LOW_DEG    12.0f
#define HEADING_PRIORITY_HIGH_DEG   30.0f
#define STEER_SLEW_RATE_PER_SEC     450.0f
#define STEER_FILTER_ALPHA          0.28f
#define HEADING_SLOWDOWN_GAIN       10.0f


//기본 전진 속도
int base_speed = 150;

//조향량 제한
#define STEER_LIMIT             85.0f

// pure-pursuit error가 커지는 급커브에서는 기본 PWM을 낮춰 흔들림과 이탈을 줄인다.
#define CURVE_SLOWDOWN_GAIN     0.35f
#define TRACKING_BASE_MIN       MIN_DRIVE_PWM

//perception_node에서 target error가 끊기면 정지하는 시간.
#define TARGET_TIMEOUT_MS       250


/*
 * 라인을 잠깐 놓쳤을 때 바로 멈추지 않고,
 * 마지막 error를 0으로 감쇠시키며 복구 조향하는 시간.
 */
#define LOST_RECOVERY_BASE_PWM  140
#define LOST_RECOVERY_TIME_MS   400


/* =====================
 * Helpers
 * ===================== */
#define RCCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if ((temp_rc != RCL_RET_OK)) { \
    printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
    vTaskDelete(NULL); \
  } \
}

#define RCSOFTCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if ((temp_rc != RCL_RET_OK)) { \
    printf("Soft fail on line %d: %d\n", __LINE__, (int)temp_rc); \
  } \
}

#define RMWCHECK(fn) { \
  rmw_ret_t temp_rc = fn; \
  if ((temp_rc != RMW_RET_OK)) { \
    printf("Failed RMW status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
    vTaskDelete(NULL); \
  } \
}

/* =====================
 * micro-ROS handles
 * ===================== */
rcl_subscription_t search_cmd_subscriber;
rcl_subscription_t lateral_error_subscriber;
rcl_subscription_t heading_error_subscriber;
rcl_subscription_t lookahead_error_subscriber;

rcl_publisher_t debug_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t esp_status_publisher;

rcl_timer_t control_timer;
rcl_timer_t debug_timer;
rcl_timer_t imu_timer;

rclc_executor_t executor;
rcl_node_t node;

std_msgs__msg__Float32 lateral_error_msg;
std_msgs__msg__Float32 heading_error_msg;
std_msgs__msg__Float32 lookahead_error_msg;
std_msgs__msg__Int32 search_cmd_msg;
std_msgs__msg__String debug_msg;
std_msgs__msg__String imu_msg;
std_msgs__msg__String esp_status_msg;

/* =====================
 * Control state
 * ===================== */
float camera_heading_error = 0.0f;
float last_valid_error = 0.0f;

// Split path-control errors from perception_node.
// Units: lateral [px], heading/lookahead [deg].
volatile float path_lateral_error_px = 0.0f;
volatile float path_heading_error_deg = 0.0f;
volatile float path_lookahead_error_deg = 0.0f;
volatile bool lateral_error_valid = false;
volatile bool heading_error_valid = false;
volatile bool lookahead_error_valid = false;
uint32_t last_lateral_error_time = 0;
uint32_t last_heading_error_time = 0;
uint32_t last_lookahead_error_time = 0;

float last_heading_priority = 0.0f;
float filtered_steer = 0.0f;
int last_active_base_speed = 0;

float integral_err = 0.0f;
float prev_err = 0.0f;

uint32_t lost_recovery_start_time = 0;
bool lost_recovery_active = false;

int last_left_pwm = 0;
int last_right_pwm = 0;
float last_steer = 0.0f;
float last_deriv_err = 0.0f;

bool was_stopped = true;
bool start_kick_active = false;
uint32_t start_kick_start_time = 0;

volatile bool target_error_valid = false;

volatile int32_t search_cmd = 0;
uint32_t last_search_cmd_time = 0;

uint32_t last_target_msg_time = 0;
uint32_t last_control_time = 0;

/* =====================
 * ESP status metrics
 * ===================== */
volatile float uros_loop_dt_ms = 0.0f;
volatile float uros_loop_max_dt_ms = 0.0f;
volatile float uros_loop_rate_hz = 0.0f;
volatile uint32_t uros_loop_count = 0;
int64_t uros_last_loop_time_us = 0;
uint32_t uros_last_rate_time_ms = 0;

char debug_str_buf[256];
char imu_str_buf[192];
char esp_status_str_buf[256];

/* =====================
 * Common functions
 * ===================== */
static int clamp_int(int value, int min_value, int max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static float clamp_float(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static float smoothstep01(float x)
{
    x = clamp_float(x, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

static float slew_limit_float(float target, float current, float max_delta)
{
    float delta = target - current;
    delta = clamp_float(delta, -max_delta, max_delta);
    return current + delta;
}

static int apply_min_drive_pwm(int pwm)
{
    /*
     * pwm이 0이면 정지.
     * pwm이 1~129이면 모터가 안 도니까 130으로 보정.
     */
    if (pwm <= 0) {
        return 0;
    }

    if (pwm < MIN_DRIVE_PWM) {
        return MIN_DRIVE_PWM;
    }

    return pwm;
}

/* =====================
 * MPU6050 functions
 * ===================== */
static esp_err_t i2c_master_init_mpu6050(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = i2c_driver_install(
        I2C_MASTER_NUM,
        conf.mode,
        0,
        0,
        0
    );

    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }

    return ESP_OK;
}

static esp_err_t mpu6050_write_byte(uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};

    return i2c_master_write_to_device(
        I2C_MASTER_NUM,
        MPU6050_ADDR,
        write_buf,
        sizeof(write_buf),
        pdMS_TO_TICKS(100)
    );
}

static esp_err_t mpu6050_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(
        I2C_MASTER_NUM,
        MPU6050_ADDR,
        &reg_addr,
        1,
        data,
        len,
        pdMS_TO_TICKS(100)
    );
}

static esp_err_t mpu6050_read_whoami(uint8_t *whoami)
{
    return mpu6050_read_bytes(MPU6050_WHO_AM_I, whoami, 1);
}

static esp_err_t mpu6050_init_sensor(void)
{
    esp_err_t ret;

    // Wake up MPU6050
    ret = mpu6050_write_byte(MPU6050_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    // Gyro full scale: ±250 deg/s
    ret = mpu6050_write_byte(MPU6050_GYRO_CONFIG, 0x00);
    if (ret != ESP_OK) {
        return ret;
    }

    // Accel full scale: ±2g
    ret = mpu6050_write_byte(MPU6050_ACCEL_CONFIG, 0x00);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    return ESP_OK;
}

static int16_t combine_high_low(uint8_t high, uint8_t low)
{
    return (int16_t)((high << 8) | low);
}

static esp_err_t mpu6050_read_scaled(
    float *ax,
    float *ay,
    float *az,
    float *gx,
    float *gy,
    float *gz
)
{
    uint8_t raw_data[14];

    esp_err_t ret = mpu6050_read_bytes(
        MPU6050_ACCEL_XOUT_H,
        raw_data,
        14
    );

    if (ret != ESP_OK) {
        return ret;
    }

    int16_t raw_ax = combine_high_low(raw_data[0], raw_data[1]);
    int16_t raw_ay = combine_high_low(raw_data[2], raw_data[3]);
    int16_t raw_az = combine_high_low(raw_data[4], raw_data[5]);

    int16_t raw_gx = combine_high_low(raw_data[8], raw_data[9]);
    int16_t raw_gy = combine_high_low(raw_data[10], raw_data[11]);
    int16_t raw_gz = combine_high_low(raw_data[12], raw_data[13]);

    /*
     * accel: g 단위
     * gyro : deg/s 단위
     */
    *ax = (float)raw_ax / ACCEL_SCALE;
    *ay = (float)raw_ay / ACCEL_SCALE;
    *az = (float)raw_az / ACCEL_SCALE;

    *gx = (float)raw_gx / GYRO_SCALE;
    *gy = (float)raw_gy / GYRO_SCALE;
    *gz = (float)raw_gz / GYRO_SCALE;

    return ESP_OK;
}


/* =====================
 * OTA server discovery
 * ===================== */
bool find_ota_server(void)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket");
        return false;
    }

    struct sockaddr_in dest_addr;
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(19700);
    dest_addr.sin_addr.s_addr = inet_addr("255.255.255.255");

    int broadcast = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    struct timeval tv = {
        .tv_sec = 3,
        .tv_usec = 0
    };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    const char *msg = "WHO_IS_OTA_SERVER";
    sendto(sock, msg, strlen(msg), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    ESP_LOGI(TAG, "Broadcasting to find OTA server...");

    char recv_buf[128];
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);
    int len = recvfrom(sock, recv_buf, sizeof(recv_buf) - 1, 0,
                       (struct sockaddr *)&source_addr, &socklen);

    close(sock);

    if (len > 0) {
        recv_buf[len] = 0;

        if (strncmp(recv_buf, "I_AM_OTA_SERVER:", 16) == 0) {
            const char *server_ip = recv_buf + 16;

            int written = snprintf(
                dynamic_ota_url,
                sizeof(dynamic_ota_url),
                "https://%.64s:8000/firmware.bin",
                server_ip
            );

            if (written < 0 || written >= (int)sizeof(dynamic_ota_url)) {
                ESP_LOGE(TAG, "OTA URL truncated or formatting failed");
                return false;
            }

            ESP_LOGI(TAG, "Found OTA Server! URL: %s", dynamic_ota_url);
            return true;
        }
    }

    ESP_LOGE(TAG, "OTA server not found");
    return false;
}

/* =====================
 * Motor control
 * ===================== */
static void motor_gpio_init(void)
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << IN1) | (1ULL << IN2) | (1ULL << IN3) | (1ULL << IN4),
        .pull_down_en = 0,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel_a = {
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL_A,
        .timer_sel  = LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = LEDC_OUTPUT_IO_A,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&ledc_channel_a);

    ledc_channel_config_t ledc_channel_b = {
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL_B,
        .timer_sel  = LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = LEDC_OUTPUT_IO_B,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&ledc_channel_b);
}

static void set_motor_single_forward_only(int pwm, gpio_num_t in1, gpio_num_t in2, ledc_channel_t channel)
{
    int duty = clamp_int(pwm, 0, 255);

    if (duty > 0) {
        gpio_set_level(in1, 1);
        gpio_set_level(in2, 0);
        ledc_set_duty(LEDC_MODE, channel, duty);
    } else {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 0);
        ledc_set_duty(LEDC_MODE, channel, 0);
    }

    ledc_update_duty(LEDC_MODE, channel);
}

static void set_motor_single_signed(int pwm, gpio_num_t in1, gpio_num_t in2, ledc_channel_t channel)
{
    int duty = pwm;

    if (duty > 255) duty = 255;
    if (duty < -255) duty = -255;

    if (duty > 0) {
        gpio_set_level(in1, 1);
        gpio_set_level(in2, 0);
        ledc_set_duty(LEDC_MODE, channel, duty);
    } else if (duty < 0) {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 1);
        ledc_set_duty(LEDC_MODE, channel, -duty);
    } else {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 0);
        ledc_set_duty(LEDC_MODE, channel, 0);
    }

    ledc_update_duty(LEDC_MODE, channel);
}

static void set_motor_pwm_signed(int left_pwm, int right_pwm)
{
    set_motor_single_signed(left_pwm, IN1, IN2, LEDC_CHANNEL_A);
    set_motor_single_signed(right_pwm, IN3, IN4, LEDC_CHANNEL_B);
}

static void set_motor_pwm_forward_only(int left_pwm, int right_pwm)
{
    left_pwm = clamp_int(left_pwm, 0, 255);
    right_pwm = clamp_int(right_pwm, 0, 255);

    left_pwm = apply_min_drive_pwm(left_pwm);
    right_pwm = apply_min_drive_pwm(right_pwm);

    set_motor_single_forward_only(left_pwm, IN1, IN2, LEDC_CHANNEL_A);
    set_motor_single_forward_only(right_pwm, IN3, IN4, LEDC_CHANNEL_B);
}

static void stop_all_motors(void)
{
    bool was_moving = (last_left_pwm != 0 || last_right_pwm != 0);

    last_left_pwm = 0;
    last_right_pwm = 0;
    last_steer = 0.0f;
    filtered_steer = 0.0f;
    last_heading_priority = 0.0f;

    was_stopped = true;
    start_kick_active = false;
    start_kick_start_time = 0;

    lost_recovery_active = false;
    lost_recovery_start_time = 0;

    set_motor_single_forward_only(0, IN1, IN2, LEDC_CHANNEL_A);
    set_motor_single_forward_only(0, IN3, IN4, LEDC_CHANNEL_B);

    if (was_moving) {
        ESP_LOGW(TAG, "Motors stopped");
    }
}

/* =====================
 * OTA button
 * ===================== */
static void ota_button_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << OTA_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

static void ota_button_task(void *arg)
{
    (void)arg;
    int last_state = 1;

    while (1) {
        int state = gpio_get_level(OTA_BUTTON_GPIO);

        if (last_state == 1 && state == 0 && !ota_in_progress) {
            ESP_LOGW(TAG, "OTA button pressed -> OTA requested");
            ota_requested = true;
        }

        last_state = state;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* =====================
 * micro-ROS callbacks
 * ===================== */
void lateral_error_callback(const void * msgin)
{
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;

    if (ota_in_progress) {
        stop_all_motors();
        return;
    }

    path_lateral_error_px = msg->data;
    last_lateral_error_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    last_target_msg_time = last_lateral_error_time;
    lateral_error_valid = true;
    target_error_valid = lateral_error_valid && heading_error_valid && lookahead_error_valid;
}

void heading_error_callback(const void * msgin)
{
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;

    if (ota_in_progress) {
        stop_all_motors();
        return;
    }

    path_heading_error_deg = msg->data;
    last_heading_error_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    last_target_msg_time = last_heading_error_time;
    heading_error_valid = true;
    target_error_valid = lateral_error_valid && heading_error_valid && lookahead_error_valid;
}

void lookahead_error_callback(const void * msgin)
{
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;

    if (ota_in_progress) {
        stop_all_motors();
        return;
    }

    path_lookahead_error_deg = msg->data;
    camera_heading_error = path_lookahead_error_deg;  // debug compatibility: old CamErr field now mirrors LookErr
    last_lookahead_error_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    last_target_msg_time = last_lookahead_error_time;
    lookahead_error_valid = true;
    target_error_valid = lateral_error_valid && heading_error_valid && lookahead_error_valid;
}

void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) last_call_time;

    if (timer == NULL || ota_in_progress) {
        return;
    }

    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    /*
     * TODO:
     * 아래 값들은 실제 MPU6050 read 함수로 교체해야 함.
     * 예:
     *   ax, ay, az = accelerometer
     *   gx, gy, gz = gyroscope
     */
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;

    esp_err_t imu_ret = mpu6050_read_scaled(&ax, &ay, &az, &gx, &gy, &gz);

    if (imu_ret != ESP_OK) {
        snprintf(
            imu_str_buf,
            sizeof(imu_str_buf),
            "T:%lu | IMU_ERR:%s",
            (unsigned long)now_ms,
            esp_err_to_name(imu_ret)
        );

        imu_msg.data.data = imu_str_buf;
        imu_msg.data.size = strlen(imu_str_buf);
        imu_msg.data.capacity = sizeof(imu_str_buf);

        RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
        return;
    }

    snprintf(
        imu_str_buf,
        sizeof(imu_str_buf),
        "T:%lu | Ax:%.4f | Ay:%.4f | Az:%.4f | Gx:%.4f | Gy:%.4f | Gz:%.4f",
        (unsigned long)now_ms,
        ax,
        ay,
        az,
        gx,
        gy,
        gz
    );

    imu_msg.data.data = imu_str_buf;
    imu_msg.data.size = strlen(imu_str_buf);
    imu_msg.data.capacity = sizeof(imu_str_buf);

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
}


void search_cmd_callback(const void * msgin)
{
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;

    if (ota_in_progress) {
        stop_all_motors();
        return;
    }

    search_cmd = msg->data;
    last_search_cmd_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

void control_loop_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) last_call_time;

    if (timer == NULL || ota_in_progress) {
        return;
    }

    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

    /*
     * Marker recovery mode.
     *
     * /real/perception/search_cmd:
     *   10 -> marker 10만 보임
     *   15 -> marker 15만 보임
     *
     * 이 모드는 split path error timeout보다 우선한다.
     */
    bool search_cmd_recent = (now - last_search_cmd_time) <= SEARCH_CMD_TIMEOUT_MS;
    
    if (!search_cmd_recent) {
    search_cmd = 0;
    }

    if (search_cmd_recent && search_cmd == 10) {
        int left_pwm = SEARCH_TURN_PWM;
        int right_pwm = -SEARCH_TURN_PWM;

        last_steer = 0.0f;
        last_left_pwm = left_pwm;
        last_right_pwm = right_pwm;

        was_stopped = false;
        start_kick_active = false;

        last_control_time = now;
        prev_err = camera_heading_error;
        last_deriv_err = 0.0f;

        filtered_steer = 0.0f;
        last_heading_priority = 0.0f;

        set_motor_pwm_signed(left_pwm, right_pwm);
        return;
    }

    if (search_cmd_recent && search_cmd == 15) {
        int left_pwm = -SEARCH_TURN_PWM;
        int right_pwm = SEARCH_TURN_PWM;

        last_steer = 0.0f;
        last_left_pwm = left_pwm;
        last_right_pwm = right_pwm;

        was_stopped = false;
        start_kick_active = false;

        last_control_time = now;
        prev_err = camera_heading_error;
        last_deriv_err = 0.0f;

        filtered_steer = 0.0f;
        last_heading_priority = 0.0f;

        set_motor_pwm_signed(left_pwm, right_pwm);
        return;
    }

    /*
     * split path error 3종을 한 번도 모두 받은 적이 없으면 lost recovery 하면 안 됨.
     * 이때는 그냥 정지.
     */
    if (!target_error_valid) {
        stop_all_motors();

        integral_err = 0.0f;
        prev_err = 0.0f;
        last_deriv_err = 0.0f;

        lost_recovery_active = false;
        lost_recovery_start_time = 0;

        last_control_time = now;
        return;
    }
    /*
    * lost recovery가 이미 시작된 상태라면,
    * 새 split path error가 들어와도 recovery가 끝날 때까지 정상 tracking으로 복귀하지 않는다.
    */
    if (lost_recovery_active) {
        uint32_t lost_elapsed = now - lost_recovery_start_time;

        if (lost_elapsed <= LOST_RECOVERY_TIME_MS) {
            float alpha = 1.0f - ((float)lost_elapsed / (float)LOST_RECOVERY_TIME_MS);

            if (alpha < 0.0f) alpha = 0.0f;
            if (alpha > 1.0f) alpha = 1.0f;

            float recovery_error = last_valid_error * alpha;

            float recovery_steer = STEER_SIGN * (Kp * recovery_error);
            recovery_steer = clamp_float(recovery_steer, -STEER_LIMIT, STEER_LIMIT);

            int left_pwm = LOST_RECOVERY_BASE_PWM - (int)recovery_steer;
            int right_pwm = LOST_RECOVERY_BASE_PWM + (int)recovery_steer;

            left_pwm = clamp_int(left_pwm, 1, 255);
            right_pwm = clamp_int(right_pwm, 1, 255);

            left_pwm = apply_min_drive_pwm(left_pwm);
            right_pwm = apply_min_drive_pwm(right_pwm);

            last_steer = recovery_steer;
            last_left_pwm = left_pwm;
            last_right_pwm = right_pwm;

            last_control_time = now;
            prev_err = recovery_error;
            last_deriv_err = 0.0f;

            set_motor_single_forward_only(left_pwm, IN1, IN2, LEDC_CHANNEL_A);
            set_motor_single_forward_only(right_pwm, IN3, IN4, LEDC_CHANNEL_B);

            return;
        }

        /*
        * recovery 완료.
        * 이제 정상 tracking으로 복귀 허용.
        */
        lost_recovery_active = false;
        lost_recovery_start_time = 0;

        integral_err = 0.0f;
        prev_err = camera_heading_error;
        last_deriv_err = 0.0f;

        bool split_recent_after_recovery =
            lateral_error_valid && heading_error_valid && lookahead_error_valid &&
            ((now - last_lateral_error_time) <= TARGET_TIMEOUT_MS) &&
            ((now - last_heading_error_time) <= TARGET_TIMEOUT_MS) &&
            ((now - last_lookahead_error_time) <= TARGET_TIMEOUT_MS);

        if (!split_recent_after_recovery) {
            target_error_valid = false;
            stop_all_motors();
            last_control_time = now;
            return;
        }
    }
    /*
     * split path error 3종 중 하나라도 일정 시간 이상 안 들어오면 lost recovery.
     */
    bool split_error_timeout =
        !(lateral_error_valid && heading_error_valid && lookahead_error_valid) ||
        ((now - last_lateral_error_time) > TARGET_TIMEOUT_MS) ||
        ((now - last_heading_error_time) > TARGET_TIMEOUT_MS) ||
        ((now - last_lookahead_error_time) > TARGET_TIMEOUT_MS);

    if (split_error_timeout) {
        /*
        * timeout 순간에 recovery 시작.
        * 실제 recovery PWM 출력은 위 lost_recovery_active 블록에서 담당.
        */
        lost_recovery_active = true;
        lost_recovery_start_time = now;

        integral_err = 0.0f;
        prev_err = last_valid_error;
        last_deriv_err = 0.0f;

        last_control_time = now;
        return;
    }

    /*
     * 정상 tracking 상태.
     */
    float dt = (now - last_control_time) / 1000.0f;
    last_control_time = now;

    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.01f;
    }

    /*
     * Use split path errors from perception_node:
     *   /real/path/lateral_error   [px]
     *   /real/path/heading_error   [deg]
     *   /real/path/lookahead_error [deg]
     *
     * Heading priority is blended smoothly. When heading error is small,
     * lateral + heading + lookahead are balanced. When heading error is large,
     * the controller gradually becomes heading-first, without a hard mode switch.
     */
    bool split_error_recent =
        lateral_error_valid && heading_error_valid && lookahead_error_valid &&
        ((now - last_lateral_error_time) <= TARGET_TIMEOUT_MS) &&
        ((now - last_heading_error_time) <= TARGET_TIMEOUT_MS) &&
        ((now - last_lookahead_error_time) <= TARGET_TIMEOUT_MS);

    if (!split_error_recent) {
        /*
         * /real/target_heading_error fallback을 제거했으므로,
         * split path error 3개 중 하나라도 오래되면 새 조향을 계산하지 않는다.
         * 대신 마지막 정상 steer-equivalent error로 짧은 lost recovery에 들어간다.
         */
        lost_recovery_active = true;
        lost_recovery_start_time = now;

        integral_err = 0.0f;
        prev_err = last_valid_error;
        last_deriv_err = 0.0f;

        last_control_time = now;
        return;
    }

    float e_y = path_lateral_error_px;
    float e_theta = path_heading_error_deg;
    float e_look = path_lookahead_error_deg;

    float abs_heading = fabsf(e_theta);
    float priority_raw = (abs_heading - HEADING_PRIORITY_LOW_DEG) /
                         (HEADING_PRIORITY_HIGH_DEG - HEADING_PRIORITY_LOW_DEG);
    float heading_priority = smoothstep01(priority_raw);

    float normal_steer =
        (Ky * e_y) +
        (Ktheta * e_theta) +
        (Klook * e_look);

    float heading_steer = Ktheta_high * e_theta;

    float steer_cmd = ((1.0f - heading_priority) * normal_steer) +
                      (heading_priority * heading_steer);

    /* For lost recovery/debug: keep an equivalent scalar error. */
    float error = steer_cmd;
    integral_err = 0.0f;
    last_deriv_err = 0.0f;
    prev_err = error;
    last_valid_error = error;

    steer_cmd = STEER_SIGN * steer_cmd;
    steer_cmd = clamp_float(steer_cmd, -STEER_LIMIT, STEER_LIMIT);

    /* Low-pass + slew-rate limit for smoother motion. */
    float filtered_target =
        (STEER_FILTER_ALPHA * steer_cmd) +
        ((1.0f - STEER_FILTER_ALPHA) * filtered_steer);

    float max_delta = STEER_SLEW_RATE_PER_SEC * dt;
    filtered_steer = slew_limit_float(filtered_target, filtered_steer, max_delta);

    float steer = clamp_float(filtered_steer, -STEER_LIMIT, STEER_LIMIT);
    last_heading_priority = heading_priority;

    /* Smoothly slow down in curves and during heading-priority correction. */
    int active_base_speed = base_speed
        - (int)(CURVE_SLOWDOWN_GAIN * fabsf(steer))
        - (int)(HEADING_SLOWDOWN_GAIN * heading_priority);
    active_base_speed = clamp_int(active_base_speed, TRACKING_BASE_MIN, base_speed);
    last_active_base_speed = active_base_speed;

    int left_pwm = active_base_speed - (int)steer;
    int right_pwm = active_base_speed + (int)steer;

    /*
     * target이 valid한 정상 주행 중에는 한쪽 PWM이 0으로 꺼지지 않게 1~255로 제한.
     * 이후 MIN_DRIVE_PWM 적용.
     */
    left_pwm = clamp_int(left_pwm, 1, 255);
    right_pwm = clamp_int(right_pwm, 1, 255);

    left_pwm = apply_min_drive_pwm(left_pwm);
    right_pwm = apply_min_drive_pwm(right_pwm);

    /*
     * 출발 킥.
     */
    bool command_moving = (left_pwm > 0 || right_pwm > 0);

    if (command_moving && was_stopped && !start_kick_active) {
        start_kick_active = true;
        start_kick_start_time = now;
    }

    if (start_kick_active) {
        uint32_t kick_elapsed = now - start_kick_start_time;

        if (kick_elapsed <= START_KICK_TIME_MS) {
            if (left_pwm > 0 && left_pwm < START_KICK_PWM) {
                left_pwm = START_KICK_PWM;
            }

            if (right_pwm > 0 && right_pwm < START_KICK_PWM) {
                right_pwm = START_KICK_PWM;
            }
        } else {
            start_kick_active = false;
            was_stopped = false;
        }
    }

    if (!command_moving) {
        was_stopped = true;
        start_kick_active = false;
        start_kick_start_time = 0;
    } else if (!start_kick_active) {
        was_stopped = false;
    }

    lost_recovery_active = false;
    lost_recovery_start_time = 0;

    last_steer = steer;
    last_left_pwm = left_pwm;
    last_right_pwm = right_pwm;

    set_motor_single_forward_only(left_pwm, IN1, IN2, LEDC_CHANNEL_A);
    set_motor_single_forward_only(right_pwm, IN3, IN4, LEDC_CHANNEL_B);
}

static void get_wifi_status_text(char *buf, size_t buf_len)
{
    if (buf == NULL || buf_len == 0) {
        return;
    }

    wifi_ap_record_t ap_info;
    esp_err_t wifi_ret = esp_wifi_sta_get_ap_info(&ap_info);

    if (wifi_ret != ESP_OK) {
        snprintf(buf, buf_len, "WiFi:0 | RSSI: | CH: | IP:");
        return;
    }

    char ip_str[16] = "";
    esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta_netif != NULL) {
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(sta_netif, &ip_info) == ESP_OK) {
            snprintf(
                ip_str,
                sizeof(ip_str),
                IPSTR,
                IP2STR(&ip_info.ip)
            );
        }
    }

    snprintf(
        buf,
        buf_len,
        "WiFi:1 | RSSI:%d | CH:%d | IP:%s",
        (int)ap_info.rssi,
        (int)ap_info.primary,
        ip_str
    );
}

void debug_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) last_call_time;

    if (timer == NULL || ota_in_progress) {
        return;
    }

    uint32_t age_ms = 0;
    if (target_error_valid) {
        age_ms = xTaskGetTickCount() * portTICK_PERIOD_MS - last_target_msg_time;
    }

    snprintf(
        debug_str_buf,
        sizeof(debug_str_buf),
        "CamErr:%.2f | LastErr:%.2f | Lat:%.2f | Head:%.2f | Look:%.2f | HPrio:%.2f | Steer:%.2f | L:%d | R:%d | Age:%lu | Valid:%d | Base:%d | ABase:%d | Min:%d | Kick:%d | Search:%ld | Lost:%d",
        camera_heading_error,
        last_valid_error,
        path_lateral_error_px,
        path_heading_error_deg,
        path_lookahead_error_deg,
        last_heading_priority,
        last_steer,
        last_left_pwm,
        last_right_pwm,
        (unsigned long)age_ms,
        target_error_valid ? 1 : 0,
        base_speed,
        last_active_base_speed,
        MIN_DRIVE_PWM,
        start_kick_active ? 1 : 0,
        (long)search_cmd,
        lost_recovery_active ? 1 : 0
    );

    debug_msg.data.data = debug_str_buf;
    debug_msg.data.size = strlen(debug_str_buf);
    debug_msg.data.capacity = sizeof(debug_str_buf);

    RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));

    char wifi_status[96];
    get_wifi_status_text(wifi_status, sizeof(wifi_status));

    snprintf(
        esp_status_str_buf,
        sizeof(esp_status_str_buf),
        "T:%lu | LoopRate:%.2f | LoopDt:%.2f | MaxLoopDt:%.2f | %s | FreeHeap:%lu | MinFreeHeap:%lu",
        (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS),
        uros_loop_rate_hz,
        uros_loop_dt_ms,
        uros_loop_max_dt_ms,
        wifi_status,
        (unsigned long)esp_get_free_heap_size(),
        (unsigned long)esp_get_minimum_free_heap_size()
    );

    esp_status_msg.data.data = esp_status_str_buf;
    esp_status_msg.data.size = strlen(esp_status_str_buf);
    esp_status_msg.data.capacity = sizeof(esp_status_str_buf);

    RCSOFTCHECK(rcl_publish(&esp_status_publisher, &esp_status_msg, NULL));
}

/* =====================
 * micro-ROS task
 * ===================== */
void micro_ros_task(void * arg)
{
    (void)arg;

    ESP_LOGI(TAG, "micro_ros_task started");

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rmw_ret_t rmw_rc;
    rcl_ret_t rc;

    rc = rcl_init_options_init(&init_options, allocator);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "rcl_init_options_init failed: %ld", (long)rc);
        vTaskDelete(NULL);
    }

    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    rmw_rc = rmw_uros_options_set_udp_address(
        CONFIG_MICRO_ROS_AGENT_IP,
        CONFIG_MICRO_ROS_AGENT_PORT,
        rmw_options
    );
    if (rmw_rc != RMW_RET_OK) {
        ESP_LOGE(TAG, "rmw_uros_options_set_udp_address failed: %ld", (long)rmw_rc);
        vTaskDelete(NULL);
    }

    rc = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "rclc_support_init_with_options failed: %ld", (long)rc);
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "support init ok");

    node = rcl_get_zero_initialized_node();
    search_cmd_subscriber = rcl_get_zero_initialized_subscription();
    lateral_error_subscriber = rcl_get_zero_initialized_subscription();
    heading_error_subscriber = rcl_get_zero_initialized_subscription();
    lookahead_error_subscriber = rcl_get_zero_initialized_subscription();

    debug_publisher = rcl_get_zero_initialized_publisher();
    imu_publisher = rcl_get_zero_initialized_publisher();
    esp_status_publisher = rcl_get_zero_initialized_publisher();

    control_timer = rcl_get_zero_initialized_timer();
    debug_timer = rcl_get_zero_initialized_timer();
    imu_timer = rcl_get_zero_initialized_timer();

    executor = rclc_executor_get_zero_initialized_executor();

    rmw_ret_t ping_rc = rmw_uros_ping_agent(1000, 3);
    ESP_LOGI(TAG, "ping agent rc: %ld", (long)ping_rc);

    rcl_node_options_t node_ops = rcl_node_get_default_options();
    node_ops.enable_rosout = false;
    node_ops.use_global_arguments = false;

    rc = rcl_node_init(&node, "esp32node", "/", &support.context, &node_ops);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "rcl_node_init failed: %ld", (long)rc);
        ESP_LOGE(TAG, "rcl error string: %s", rcl_get_error_string().str);
        rcl_reset_error();
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "node init ok");

    rc = rclc_subscription_init_best_effort(
        &search_cmd_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/real/perception/search_cmd"
    );
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "search_cmd subscription init failed: %ld", (long)rc);
        ESP_LOGE(TAG, "rcl error string: %s", rcl_get_error_string().str);
        rcl_reset_error();
        vTaskDelete(NULL);
    }

    rc = rclc_subscription_init_best_effort(
        &lateral_error_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/real/path/lateral_error"
    );
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "lateral_error subscription init failed: %ld", (long)rc);
        ESP_LOGE(TAG, "rcl error string: %s", rcl_get_error_string().str);
        rcl_reset_error();
        vTaskDelete(NULL);
    }

    rc = rclc_subscription_init_best_effort(
        &heading_error_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/real/path/heading_error"
    );
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "heading_error subscription init failed: %ld", (long)rc);
        ESP_LOGE(TAG, "rcl error string: %s", rcl_get_error_string().str);
        rcl_reset_error();
        vTaskDelete(NULL);
    }

    rc = rclc_subscription_init_best_effort(
        &lookahead_error_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/real/path/lookahead_error"
    );
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "lookahead_error subscription init failed: %ld", (long)rc);
        ESP_LOGE(TAG, "rcl error string: %s", rcl_get_error_string().str);
        rcl_reset_error();
        vTaskDelete(NULL);
    }

    rc = rclc_publisher_init_best_effort(
        &debug_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/esp32/debug_status"
    );
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "publisher init failed: %ld", (long)rc);
        ESP_LOGE(TAG, "rcl error string: %s", rcl_get_error_string().str);
        rcl_reset_error();
        vTaskDelete(NULL);
    }

    rc = rclc_publisher_init_best_effort(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/esp32/imu_debug"
    );
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "imu publisher init failed: %ld", (long)rc);
        ESP_LOGE(TAG, "rcl error string: %s", rcl_get_error_string().str);
        rcl_reset_error();
        vTaskDelete(NULL);
    }

    rc = rclc_publisher_init_best_effort(
        &esp_status_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/esp32/status"
    );
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "esp_status publisher init failed: %ld", (long)rc);
        ESP_LOGE(TAG, "rcl error string: %s", rcl_get_error_string().str);
        rcl_reset_error();
        vTaskDelete(NULL);
    }

    rc = rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(10),
        control_loop_callback
    );
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "control timer init failed: %ld", (long)rc);
        vTaskDelete(NULL);
    }

    rc = rclc_timer_init_default(
        &debug_timer,
        &support,
        RCL_MS_TO_NS(100),
        debug_timer_callback
    );
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "debug timer init failed: %ld", (long)rc);
        vTaskDelete(NULL);
    }

    rc = rclc_timer_init_default(
        &imu_timer,
        &support,
        RCL_MS_TO_NS(IMU_LOG_PERIOD_MS),
        imu_timer_callback
    );
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "imu timer init failed: %ld", (long)rc);
        vTaskDelete(NULL);
    }

    rc = rclc_executor_init(&executor, &support.context, 8, &allocator);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "executor init failed: %ld", (long)rc);
        vTaskDelete(NULL);
    }

    rc = rclc_executor_add_timer(&executor, &control_timer);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "add control timer failed: %ld", (long)rc);
        vTaskDelete(NULL);
    }

    rc = rclc_executor_add_timer(&executor, &debug_timer);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "add debug timer failed: %ld", (long)rc);
        vTaskDelete(NULL);
    }

    rc = rclc_executor_add_timer(&executor, &imu_timer);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "add imu timer failed: %ld", (long)rc);
        vTaskDelete(NULL);
    }

    rc = rclc_executor_add_subscription(
        &executor,
        &search_cmd_subscriber,
        &search_cmd_msg,
        &search_cmd_callback,
        ON_NEW_DATA
    );
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "add search_cmd subscription failed: %ld", (long)rc);
        vTaskDelete(NULL);
    }

    rc = rclc_executor_add_subscription(
        &executor,
        &lateral_error_subscriber,
        &lateral_error_msg,
        &lateral_error_callback,
        ON_NEW_DATA
    );
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "add lateral_error subscription failed: %ld", (long)rc);
        vTaskDelete(NULL);
    }

    rc = rclc_executor_add_subscription(
        &executor,
        &heading_error_subscriber,
        &heading_error_msg,
        &heading_error_callback,
        ON_NEW_DATA
    );
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "add heading_error subscription failed: %ld", (long)rc);
        vTaskDelete(NULL);
    }

    rc = rclc_executor_add_subscription(
        &executor,
        &lookahead_error_subscriber,
        &lookahead_error_msg,
        &lookahead_error_callback,
        ON_NEW_DATA
    );
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "add lookahead_error subscription failed: %ld", (long)rc);
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "ESP32 camera-error control node started");

    last_target_msg_time = 0;
    last_control_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    target_error_valid = false;

    camera_heading_error = 0.0f;  // debug compatibility: mirrors LookErr after first lookahead message
    integral_err = 0.0f;
    prev_err = 0.0f;
    last_deriv_err = 0.0f;

    last_left_pwm = 0;
    last_right_pwm = 0;
    last_steer = 0.0f;
    filtered_steer = 0.0f;
    last_heading_priority = 0.0f;
    last_active_base_speed = base_speed;

    lateral_error_valid = false;
    heading_error_valid = false;
    lookahead_error_valid = false;

    uros_loop_dt_ms = 0.0f;
    uros_loop_max_dt_ms = 0.0f;
    uros_loop_rate_hz = 0.0f;
    uros_loop_count = 0;
    uros_last_loop_time_us = esp_timer_get_time();
    uros_last_rate_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    while (1) {
        int64_t loop_start_us = esp_timer_get_time();

        if (uros_last_loop_time_us > 0) {
            float dt_ms = (float)(loop_start_us - uros_last_loop_time_us) / 1000.0f;
            if (dt_ms >= 0.0f && dt_ms < 10000.0f) {
                uros_loop_dt_ms = dt_ms;
                if (dt_ms > uros_loop_max_dt_ms) {
                    uros_loop_max_dt_ms = dt_ms;
                }
            }
        }
        uros_last_loop_time_us = loop_start_us;
        uros_loop_count++;

        uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        uint32_t rate_elapsed_ms = now_ms - uros_last_rate_time_ms;
        if (rate_elapsed_ms >= 1000) {
            uros_loop_rate_hz = ((float)uros_loop_count * 1000.0f) / (float)rate_elapsed_ms;
            uros_loop_count = 0;
            uros_last_rate_time_ms = now_ms;
            uros_loop_max_dt_ms = 0.0f;
        }

        if (!ota_in_progress) {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        } else {
            stop_all_motors();
            usleep(10000);
        }
    }
}

/* =====================
 * OTA logic
 * ===================== */
static void do_ota_update(void)
{
    ESP_LOGI(TAG, "Starting HTTPS OTA...");

    if (!find_ota_server()) {
        ESP_LOGE(TAG, "OTA server not found. Aborting OTA.");
        ota_in_progress = false;
        ota_requested = false;
        return;
    }

    esp_http_client_config_t http_config = {
        .url = dynamic_ota_url,
        .cert_pem = server_cert_pem_start,
        .timeout_ms = 10000,
        .keep_alive_enable = true,
    };

    esp_https_ota_config_t ota_config = {
        .http_config = &http_config,
    };

    esp_err_t ret = esp_https_ota(&ota_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "OTA successful, restarting...");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    } else {
        ESP_LOGE(TAG, "OTA failed: %s", esp_err_to_name(ret));
        ota_in_progress = false;
        ota_requested = false;
    }
}

static void ota_manager_task(void * arg)
{
    (void)arg;

    while (1) {
        if (ota_requested && !ota_in_progress) {
            ota_in_progress = true;
            ota_requested = false;

            stop_all_motors();
            vTaskDelay(pdMS_TO_TICKS(500));

            do_ota_update();

            vTaskDelay(pdMS_TO_TICKS(200));
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* =====================
 * app_main
 * ===================== */
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    motor_gpio_init();
    stop_all_motors();
    ESP_ERROR_CHECK(i2c_master_init_mpu6050());

    esp_err_t imu_init_ret = mpu6050_init_sensor();
    if (imu_init_ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 init failed: %s", esp_err_to_name(imu_init_ret));
    } else {
        ESP_LOGI(TAG, "MPU6050 init ok");
    }

    /*
     * 이번 버전에서는 IMU를 제어에는 사용하지 않는다.
     * perception에서 계산한 split path errors만으로 제어한다.
     */

    ota_button_init();

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    xTaskCreate(
        micro_ros_task,
        "uros_task",
        CONFIG_MICRO_ROS_APP_STACK,
        NULL,
        CONFIG_MICRO_ROS_APP_TASK_PRIO,
        NULL
    );

    xTaskCreate(ota_manager_task, "ota_manager_task", 8192, NULL, 4, NULL);
    xTaskCreate(ota_button_task, "ota_button_task", 2048, NULL, 2, NULL);
}