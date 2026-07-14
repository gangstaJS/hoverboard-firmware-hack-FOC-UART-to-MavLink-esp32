#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <mavlink/common/mavlink.h>

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD 115200 // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define START_FRAME 0xABCD       // [-] Start frame definition for reliable serial communication
#define TIME_SEND 100            // [ms] Sending time interval
#define SPEED_MAX_TEST 300       // [-] Maximum speed for testing
#define SPEED_STEP 20            // [-] Speed step
// #define DEBUG_RX                     // [-] Debug received data. Prints all bytes (comment-out to disable)

#define HOVER_UART_PORT UART_NUM_2
#define HOVER_SERIAL_RX_PIN 16 // ESP32 RX2  <- Hoverboard TX (use a level shifter!)
#define HOVER_SERIAL_TX_PIN 17 // ESP32 TX2  -> Hoverboard RX
#define HOVER_UART_BUF_SIZE 256

#define LED_GPIO GPIO_NUM_2 // Most ESP32 dev boards use GPIO2 for the onboard LED

static const char *TAG = "hoverboard";

#define MAV_UART UART_NUM_1

#define MAV_RX GPIO_NUM_18
#define MAV_TX GPIO_NUM_19 // ESP32 TX -> FC RX

// ########################## FRAME STRUCTS ##########################
// packed to guarantee no compiler-inserted padding, matching the wire format
typedef struct __attribute__((packed))
{
    uint16_t start;
    int16_t steer;
    int16_t speed;
    uint16_t checksum;
} SerialCommand;

typedef struct __attribute__((packed))
{
    uint16_t start;
    int16_t cmd1;
    int16_t cmd2;
    int16_t speedR_meas;
    int16_t speedL_meas;
    int16_t batVoltage;
    int16_t boardTemp;
    uint16_t cmdLed;
    uint16_t checksum;
} SerialFeedback;

static SerialFeedback Feedback;
static SerialFeedback NewFeedback;

// Receive state
static uint8_t idx = 0;        // Index for new data pointer
static uint16_t bufStartFrame; // Buffer Start Frame
static uint8_t *p;             // Pointer into NewFeedback while assembling a packet
static uint8_t incomingByte;
static uint8_t incomingBytePrev;

// ########################## HELPERS ##########################
static inline uint32_t millis(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static void mavlink_uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(MAV_UART, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(MAV_UART, &cfg));

    ESP_ERROR_CHECK(
        uart_set_pin(
            MAV_UART,
            MAV_TX,
            MAV_RX,
            UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE));
}

// ########################## MAVLINK STREAM REQUEST ##########################
static uint8_t s_fc_sysid = 1;  // updated from first FC heartbeat
static uint8_t s_fc_compid = 1; // updated from first FC heartbeat

static void mavlink_send_heartbeat(void)
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // ArduPilot only honors REQUEST_DATA_STREAM and PARAM_REQUEST_READ from
    // a sysid it considers an active GCS. Sending a heartbeat registers us.
    mavlink_msg_heartbeat_pack(
        255, 190, // our sysid/compid (GCS)
        &msg,
        MAV_TYPE_GCS,
        MAV_AUTOPILOT_INVALID,
        0, 0,
        MAV_STATE_ACTIVE);

    int len = mavlink_msg_to_send_buffer(buf, &msg);
    uart_write_bytes(MAV_UART, (const char *)buf, len);
}

static void mavlink_reset_streams(void)
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Cancel all streams so previous MAV_DATA_STREAM_ALL @ 50 Hz doesn't
    // keep saturating the link.
    mavlink_msg_request_data_stream_pack(
        255, 190, &msg,
        s_fc_sysid, s_fc_compid,
        MAV_DATA_STREAM_ALL,
        0,  // 0 Hz = stop
        0); // stop

    int len = mavlink_msg_to_send_buffer(buf, &msg);
    uart_write_bytes(MAV_UART, (const char *)buf, len);
    ESP_LOGI(TAG, "Cancelled all streams");
}

static void mavlink_read_sr4_raw_ctrl(void)
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // SERVO_OUTPUT_RAW is in STREAM_RC_CHANNELS on ArduPilot Rover -> SR4_RC_CHAN
    char param_id[16] = "SR4_RC_CHAN";
    mavlink_msg_param_request_read_pack(
        255, 190, &msg,
        s_fc_sysid, s_fc_compid,
        param_id,
        -1);

    int len = mavlink_msg_to_send_buffer(buf, &msg);
    uart_write_bytes(MAV_UART, (const char *)buf, len);
    ESP_LOGI(TAG, "PARAM_REQUEST_READ SR4_RC_CHAN");
}

static void mavlink_set_sr4_raw_ctrl(uint8_t hz)
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // PARAM_SET: force SR4_RC_CHAN=hz — SERVO_OUTPUT_RAW is in RC_CHANNELS stream on Rover.
    char param_id[16] = "SR4_RC_CHAN";
    mavlink_msg_param_set_pack(
        255, 190,
        &msg,
        s_fc_sysid, s_fc_compid,
        param_id,
        (float)hz,
        MAV_PARAM_TYPE_REAL32);

    int len = mavlink_msg_to_send_buffer(buf, &msg);
    uart_write_bytes(MAV_UART, (const char *)buf, len);
    ESP_LOGI(TAG, "PARAM_SET SR4_RC_CHAN -> %u", hz);
}

static void mavlink_request_servo_output(void)
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // SERVO_OUTPUT_RAW is in STREAM_RC_CHANNELS on ArduPilot Rover, not RAW_CONTROLLER.
    mavlink_msg_request_data_stream_pack(
        255, 190,
        &msg,
        s_fc_sysid, s_fc_compid,
        MAV_DATA_STREAM_RC_CHANNELS,
        50,
        1);

    int len = mavlink_msg_to_send_buffer(buf, &msg);
    uart_write_bytes(MAV_UART, (const char *)buf, len);
    ESP_LOGI(TAG, "REQUEST_DATA_STREAM RC_CHANNELS @ 50 Hz");
}

// ########################## SEND ##########################
static void Send(int16_t uSteer, int16_t uSpeed)
{
    SerialCommand Command;

    Command.start = (uint16_t)START_FRAME;
    Command.steer = uSteer;
    Command.speed = uSpeed;
    Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

    ESP_LOGI(TAG, "Send: steer=%d speed=%d checksum=0x%04X", uSteer, uSpeed, Command.checksum);

    int written = uart_write_bytes(HOVER_UART_PORT, (const char *)&Command, sizeof(Command));
    if (written != sizeof(Command))
    {
        ESP_LOGI(TAG, "Send: uart_write_bytes wrote %d/%d bytes", written, (int)sizeof(Command));
    }
}

static uint32_t s_last_servo_ms = 0; // timestamp of last SERVO_OUTPUT_RAW received

static void mavlink_task(void)
{
    static int16_t s_steering = 0;
    static int16_t s_throttle = 0;

    uint8_t buf[256];

    mavlink_message_t msg;
    mavlink_status_t status;

    int len = uart_read_bytes(
        MAV_UART,
        buf,
        sizeof(buf),
        pdMS_TO_TICKS(1));

    for (int i = 0; i < len; i++)
    {
        if (!mavlink_parse_char(
                MAVLINK_COMM_0,
                buf[i],
                &msg,
                &status))
        {
            continue;
        }

        switch (msg.msgid)
        {
        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
        {
            mavlink_servo_output_raw_t servo;

            mavlink_msg_servo_output_raw_decode(
                &msg,
                &servo);

            s_throttle = ((int16_t)servo.servo7_raw - 1500) * 2;
            s_steering = ((int16_t)servo.servo8_raw - 1500) * 2;

            uint32_t now_ms = millis();
            s_last_servo_ms = now_ms;

            // Measure and log actual message rate every second
            static uint32_t s_rate_count = 0;
            static uint32_t s_rate_window_ms = 0;
            s_rate_count++;
            if (s_rate_window_ms == 0)
                s_rate_window_ms = now_ms;
            uint32_t elapsed = now_ms - s_rate_window_ms;
            if (elapsed >= 1000)
            {
                ESP_LOGI(TAG, "SERVO_OUTPUT_RAW rate: %lu Hz", s_rate_count * 1000 / elapsed);
                s_rate_count = 0;
                s_rate_window_ms = now_ms;
            }

            // ESP_LOGI(TAG, "servo7=%u servo8=%u -> throttle=%d steering=%d",
            // servo.servo7_raw, servo.servo8_raw, s_throttle, s_steering);

            break;
        }

        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            // Capture FC sysid/compid the first time we see a non-GCS heartbeat
            if (msg.sysid != 255 && s_fc_sysid == 1 && msg.sysid != 1)
            {
                s_fc_sysid = msg.sysid;
                s_fc_compid = msg.compid;
                ESP_LOGI(TAG, "FC identified: sysid=%u compid=%u", s_fc_sysid, s_fc_compid);
            }
            break;
        }

        case MAVLINK_MSG_ID_COMMAND_ACK:
        {
            mavlink_command_ack_t ack;
            mavlink_msg_command_ack_decode(&msg, &ack);
            ESP_LOGI(TAG, "COMMAND_ACK cmd=%u result=%u", ack.command, ack.result);
            break;
        }

        case MAVLINK_MSG_ID_PARAM_VALUE:
        {
            mavlink_param_value_t pv;
            mavlink_msg_param_value_decode(&msg, &pv);
            // Log any SR4_ parameter so we can see the actual FC values
            if (strncmp(pv.param_id, "SR4_", 4) == 0)
                ESP_LOGI(TAG, "PARAM_VALUE %.*s = %.0f", 16, pv.param_id, pv.param_value);
            break;
        }

            // case MAVLINK_MSG_ID_HEARTBEAT:
            // {
            //     mavlink_heartbeat_t hb;
            //     mavlink_msg_heartbeat_decode(&msg, &hb);
            //     ESP_LOGI(TAG, "Heartbeat autopilot=%u type=%u", hb.autopilot, hb.type);
            //     break;
            // }
        }
    }

    Send(s_steering, s_throttle);
}

// ########################## UART INIT ##########################
static void hoverboard_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = HOVER_SERIAL_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(HOVER_UART_PORT, HOVER_UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(HOVER_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(HOVER_UART_PORT, HOVER_SERIAL_TX_PIN, HOVER_SERIAL_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

// ########################## APP MAIN (loop) ##########################
void app_main(void)
{
    hoverboard_uart_init();
    mavlink_uart_init();

    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    ESP_LOGI(TAG, "Hoverboard Serial v1.0 (ESP32 / ESP-IDF, C)");

    uint32_t iTimeSend = 0;
    int16_t iTest = 0;
    int16_t iStep = SPEED_STEP;

    // Cancel all active streams first (clears leftover MAV_DATA_STREAM_ALL @ 50 Hz),
    // then send heartbeat, set SR4_RC_CHAN, and request only RC_CHANNELS stream.
    mavlink_send_heartbeat();
    vTaskDelay(pdMS_TO_TICKS(200));
    mavlink_reset_streams();
    vTaskDelay(pdMS_TO_TICKS(200));
    mavlink_read_sr4_raw_ctrl();
    vTaskDelay(pdMS_TO_TICKS(200));
    mavlink_set_sr4_raw_ctrl(50);
    vTaskDelay(pdMS_TO_TICKS(200));
    mavlink_request_servo_output();
    uint32_t iTimeRequest = millis() + 5000;
    uint32_t iTimeHeartbeat = millis() + 1000; // send heartbeat every 1 s
    uint32_t iTimeCheck = millis() + 2000;     // check that stream arrived within 2 s

    while (1)
    {
        uint32_t timeNow = millis();

        // Warn once if no SERVO_OUTPUT_RAW arrived 2 s after the request
        if (iTimeCheck && iTimeCheck <= timeNow)
        {
            if (s_last_servo_ms == 0)
                ESP_LOGW(TAG, "No SERVO_OUTPUT_RAW received - check SR4_RAW_CTRL param");
            else
                ESP_LOGI(TAG, "Stream confirmed OK");
            iTimeCheck = 0; // check only once
        }

        if (iTimeHeartbeat <= timeNow)
        {
            mavlink_send_heartbeat();
            iTimeHeartbeat = timeNow + 1000;
        }

        if (iTimeRequest <= timeNow)
        {
            mavlink_request_servo_output();
            iTimeRequest = timeNow + 5000;
            iTimeCheck = timeNow + 2000; // re-arm check after each re-request
        }

        mavlink_task();

        // Send(0, 0);

        // uint32_t timeNow = millis();

        // Send commands at the configured interval
        // if (iTimeSend <= timeNow)
        // {
        //     iTimeSend = timeNow + TIME_SEND;

        //     // Calculate test command signal
        //     iTest += iStep;

        //     // invert step if reaching limit
        //     if (iTest >= SPEED_MAX_TEST || iTest <= -SPEED_MAX_TEST)
        //     {
        //         iStep = -iStep;
        //     }
        // }

        // Blink the LED
        // gpio_set_level(LED_GPIO, (timeNow % 2000) < 1000);

        // Yield to the scheduler / avoid a tight busy loop
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ########################## END ##########################