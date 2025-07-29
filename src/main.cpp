#include <Arduino.h>
#include <Wire.h>
// #include <SPI.h>
// #include <ISM330DHCXSensor.h>
// #include <ism330dhcx_msgs/msg/ism330dhcx_stamped.h>
#include <micro_ros_platformio.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <std_msgs/msg/string.h>
#include <uxr/client/transport.h>
#include <vl53l4cd_class.h>
#include <vl53l4cd_msgs/msg/vl53l4cd.h>
#include <vl53l4cd_msgs/msg/vl53l4cd_stamped.h>

// #include "ism330dhcx_reg.h"

// VL53L4CD
#define DEV_I2C_0 Wire
#define DEV_I2C_1 Wire1
#define SENSOR_0_ADDR 0x50
#define SENSOR_1_ADDR 0x52

typedef struct {
    VL53L4CD *psensor;  // pointer to sensor object
    TwoWire *pwire;
    int addr;
    int id;  // sensor id
    int shutdown_pin;
    int interrupt_pin;
    uint8_t range_status;
    int16_t distance_mm;
} tof_sensor_t;

VL53L4CD sensor0(&DEV_I2C_0, 21);
VL53L4CD sensor1(&DEV_I2C_1, 11);

tof_sensor_t tof_sensors[] = {
    {&sensor0, &DEV_I2C_0, SENSOR_0_ADDR, 0, 21, 20, 0, 0},
    {&sensor1, &DEV_I2C_1, SENSOR_1_ADDR, 1, 23, 22, 0, 0},
};
const uint8_t NUM_SENSORS = sizeof(tof_sensors) / sizeof(tof_sensors[0]);
volatile uint8_t interrupt_flags[] = {0, 0};
VL53L4CD_Result_t results;

// // ISM330DHCX
// #define GRAVITY 9.80665
// #define USE_SPI_INTERFACE
// #define ISM330DHCX_SPI_MOSI 26
// #define ISM330DHCX_SPI_MISO 1
// #define ISM330DHCX_SPI_SCLK 27
// #define ISM330DHCX_SPI_SSEL 0
// #define IMU_SENSOR_ODR 833.0f
// #define FIFO_SAMPLE_THRESHOLD 127
// const double SAMPLE_PERIOD = 1 / IMU_SENSOR_ODR;
// int32_t acc_data[3];
// uint16_t num_fifo_samples;
// typedef struct {
//     // ISM330DHCXSensor *psensor;
//     SPIClass *dev_interface;
//     int id;  // sensor id
//     uint8_t shutdown_pin;
//     uint8_t interrupt1;
//     uint8_t interrupt2;
// } imu_sensor_t;
// volatile bool imu_drdy_flag = false;
// volatile bool imu_fifo_full_flag = false;

// ISM330DHCXSensor imu_sensor(&SPI1, ISM330DHCX_SPI_SSEL);

// imu_sensor_t imu_sensor_info = {&imu_sensor, &SPI1, ISM330DHCX_SPI_SSEL, 28, 2, 3};

// LED config
uint32_t led_blink_start_time = millis();

/* microROS setup */
#define RCCHECK(fn)                    \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
            error_loop();              \
        }                              \
    }
#define RCSOFTCHECK(fn)                \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
            soft_error();              \
        }                              \
    }

rcl_publisher_t tof_publisher;
// rcl_publisher_t imu_publisher;
vl53l4cd_msgs__msg__Vl53l4cdStamped *p_msg_vl53l4cd_stamped;
// ism330dhcx_msgs__msg__Ism330dhcxStamped *p_msg_ism330dhcx_stamped;

// std_msgs__msg__String frame_id;
rosidl_runtime_c__String frame_id;
// rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
// rcl_timer_t timer;

// microROS system state
enum SystemState { AGENT_WAIT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } system_state;

/*******************
 * Sensors
 *******************/

void tof0_interrupt_handler() { interrupt_flags[0] = 1; }

void tof1_interrupt_handler() { interrupt_flags[1] = 1; }

// void imu_DRDY_interrupt_handler() { imu_drdy_flag = true; }

// void imu_FIFO_Full_interrupt_handler() { imu_fifo_full_flag = true; }

void error_loop() {
    // Serial.println(F("Error loop..."));
    while (true) {
        if (millis() - led_blink_start_time > 100) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            led_blink_start_time = millis();
        }
    }
}

void soft_error() {
    unsigned long start = millis();
    // unsigned long now = millis();
    while (millis() - start > 3000) {
        if (millis() - led_blink_start_time > 50) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            led_blink_start_time = millis();
        }
    }
}

/*
// void setup_imu_sensor() {
//     // Interrupts cannot be floating
//     pinMode(imu_sensor_info.interrupt1, OUTPUT);
//     pinMode(imu_sensor_info.interrupt2, OUTPUT);
//     digitalWrite(imu_sensor_info.interrupt1, LOW);
//     digitalWrite(imu_sensor_info.interrupt2, LOW);

//     // CS pin should be pulled low and sensor should be power cycled.
//     // Shutdown pin connected to PNP transistor
//     pinMode(ISM330DHCX_SPI_SSEL, OUTPUT);
//     digitalWrite(ISM330DHCX_SPI_SSEL, LOW);
//     pinMode(imu_sensor_info.shutdown_pin, OUTPUT);
//     digitalWrite(imu_sensor_info.shutdown_pin, HIGH);

//     ISM330DHCXStatusTypeDef imu_status;

//     SPISettings spi_settings(10000000, MSBFIRST, SPI_MODE3);

//     digitalWrite(ISM330DHCX_SPI_SSEL, HIGH);
//     digitalWrite(imu_sensor_info.shutdown_pin, LOW);

//     delay(100);

//     SPI1.begin();

//     delay(100);

//     imu_status = imu_sensor.begin();

//     // Configure FIFO
//     imu_sensor.FIFO_Set_Watermark_Level(1024);
//     imu_sensor.FIFO_ACC_Set_BDR(IMU_SENSOR_ODR);
//     imu_sensor.FIFO_Set_Mode(ISM330DHCX_STREAM_MODE);

//     // Configure ACC
//     imu_status = imu_sensor.GYRO_Disable();
//     imu_status = imu_sensor.ACC_Disable6DOrientation();
//     imu_status = imu_sensor.ACC_Enable();
//     imu_status = imu_sensor.ACC_SetFullScale(ISM330DHCX_2g);
//     // imu_status = imu_sensor.ACC_SetOutputDataRate(ISM330DHCX_XL_ODR_104Hz);
//     imu_status = imu_sensor.ACC_SetOutputDataRate(IMU_SENSOR_ODR);

//     // imu_status = imu_sensor.ACC_Enable_DRDY_On_INT1();
//     if (imu_status == ISM330DHCX_ERROR) {
//         error_loop();
//     }
//     // imu_sensor.DRDY_Set_Mode
//     while (true) {
//         uint8_t id;
//         imu_sensor.ReadID(&id);
//         Serial.print(F("ID: "));
//         Serial.println(id);
//         if (id == 0x6b) {
//             break;
//         }
//     }

//     // imu_sensor.FIFO_Set_INT1_FIFO_Full(1);

//     // // Change the interrupt pins back into inputs now that we've booted
//     // pinMode(imu_sensor_info.interrupt1, INPUT);
//     // pinMode(imu_sensor_info.interrupt2, INPUT);

//     // imu_sensor.FIFO_Set_INT1_FIFO_Full(1);
//     // imu_sensor.FIFO_Set_INT2_FIFO_Full(1);

//     // microROS
//     p_msg_ism330dhcx_stamped->dev_id = 0;
//     p_msg_ism330dhcx_stamped->header.frame_id.data = const_cast<char *>("imu0");
//     p_msg_ism330dhcx_stamped->header.frame_id.size = 4;
//     p_msg_ism330dhcx_stamped->header.frame_id.capacity = 5;
//     p_msg_ism330dhcx_stamped->sample_rate = IMU_SENSOR_ODR;
// }

// void read_imu_sensor_and_publish() {
//     SPISettings spi_settings(10000000, MSBFIRST, SPI_MODE3);
//     // SPI1.beginTransaction(spi_settings);
//     imu_sensor.FIFO_Get_Num_Samples(&num_fifo_samples);

//     // Serial.println(num_fifo_samples);
//     if (num_fifo_samples > FIFO_SAMPLE_THRESHOLD) {
//         // Get the time at the beginning, closest to the last reading
//         int64_t now_sec = rmw_uros_epoch_millis() / 1000;
//         p_msg_ism330dhcx_stamped->header.stamp.sec = static_cast<int32_t>(now_sec);

//         long double nanos_ = rmw_uros_epoch_nanos() / 1e9;
//         long double whole, fractional;
//         fractional = std::modf(nanos_, &whole) * 1e9;
//         p_msg_ism330dhcx_stamped->header.stamp.nanosec = fractional;

//         double *arr = new double[num_fifo_samples * 4]();

//         for (uint16_t i = 0; i < num_fifo_samples; ++i) {
//             imu_sensor.FIFO_ACC_Get_Axes(acc_data);

//             arr[i * 4] = nanos_ - (num_fifo_samples - i) * SAMPLE_PERIOD;
//             arr[i * 4 + 1] = static_cast<double>(acc_data[0]) / 1000 * GRAVITY;
//             arr[i * 4 + 2] = static_cast<double>(acc_data[1]) / 1000 * GRAVITY;
//             arr[i * 4 + 3] = static_cast<double>(acc_data[2]) / 1000 * GRAVITY;
//         }
//         p_msg_ism330dhcx_stamped->data_config.column.size = 4;
//         p_msg_ism330dhcx_stamped->data_config.row.size = num_fifo_samples;
//         p_msg_ism330dhcx_stamped->data.size = num_fifo_samples * 4;
//         p_msg_ism330dhcx_stamped->data.capacity = num_fifo_samples * 4;
//         p_msg_ism330dhcx_stamped->data.data = arr;

//         RCSOFTCHECK(rcl_publish(&imu_publisher, p_msg_ism330dhcx_stamped, NULL));

//         delete[] arr;
//     }
//     // SPI1.endTransaction()
// }

*/

void setup_tof_sensors() {
    VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;

    for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
        pinMode(tof_sensors[i].interrupt_pin, INPUT_PULLUP);

        tof_sensors[i].pwire->begin();    // begin the Wisre instance
        tof_sensors[i].psensor->begin();  // configure the sensor with a power cycle.
        status = tof_sensors[i].psensor->InitSensor(tof_sensors[i].addr);
        status = tof_sensors[i].psensor->VL53L4CD_SetRangeTiming(
            50, 0);  // highest timing budget (200) while avoiding low-power mode (0)
        status = tof_sensors[i].psensor->VL53L4CD_StartTemperatureUpdate();
        status = tof_sensors[i].psensor->VL53L4CD_StartRanging();

        if (status) {
            error_loop();
        }

        delay(10);
    }
}

void read_tof_sensors_and_publish() {
    uint8_t data_ready = 0;
    // rcl_ret_t rcl_status = RCL_RET_OK;
    VL53L4CD_ERROR data_ready_status = VL53L4CD_ERROR_NONE;
    VL53L4CD_ERROR read_status = VL53L4CD_ERROR_NONE;

    for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
        if (interrupt_flags[i]) {
            interrupt_flags[i] = 0;
            data_ready_status = tof_sensors[i].psensor->VL53L4CD_CheckForDataReady(&data_ready);

            if ((!data_ready_status) && (data_ready != 0)) {
                read_status = tof_sensors[i].psensor->VL53L4CD_ClearInterrupt();
                read_status = tof_sensors[i].psensor->VL53L4CD_GetResult(&results);

                p_msg_vl53l4cd_stamped->dev_id = tof_sensors[i].id;
                p_msg_vl53l4cd_stamped->status = results.range_status;
                p_msg_vl53l4cd_stamped->distance = results.distance_mm;
                p_msg_vl53l4cd_stamped->header.stamp.sec = rmw_uros_epoch_millis() / 1000;
                long double nanos_ = rmw_uros_epoch_nanos() / 1e9;
                long double whole, fractional;
                fractional = std::modf(nanos_, &whole) * 1e9;
                p_msg_vl53l4cd_stamped->header.stamp.nanosec = fractional;  // static_cast<uint32_t>(fractional);

                RCSOFTCHECK(rcl_publish(&tof_publisher, p_msg_vl53l4cd_stamped, NULL));
            }
            if (read_status) {
                error_loop();
            }
        }
        data_ready = 0;
    }
}

/*******************
 * microROS
 *******************/

bool create_rcl_entities() {
    /* Create the microROS entities */
    // Init allocator
    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    const char *_namespace = "microROS";
    const char *_node_name = "teensy";
    RCCHECK(rclc_node_init_default(&node, _node_name, _namespace, &support));

    // create publishers
    RCCHECK(rclc_publisher_init_default(
        &tof_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(vl53l4cd_msgs, msg, Vl53l4cdStamped), "vl53l4cd/data"));
    // RCCHECK(rclc_publisher_init_default(&imu_publisher, &node,
    //                                     ROSIDL_GET_MSG_TYPE_SUPPORT(ism330dhcx_msgs, msg, Ism330dhcxStamped),
    //                                     "ism330dhcx/data"));

    return true;
}

rcl_ret_t destroy_rcl_entities() {
    /* Destroy the microROS-related entities */
    rcl_ret_t ret_status = RMW_RET_OK;
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    // ret_status = rcl_publisher_fini(&imu_publisher, &node);
    ret_status = rcl_publisher_fini(&tof_publisher, &node);
    // rcl_timer_fini(&timer);
    // rclc_executor_fini(&executor);
    ret_status = rcl_node_fini(&node);
    ret_status = rclc_support_fini(&support);
    if (ret_status != RMW_RET_OK) {
        error_loop();
    }
    return RCL_RET_OK;
}

void execute_every_n_ms(int64_t ms, SystemState system_state) {
    /* Method for periodically pinging the micro_ros_agent */
    RCL_UNUSED(system_state);
    do {
        static volatile int64_t init = -1;
        if (init == -1) {
            init = uxr_millis();
        }
        if (uxr_millis() - init > ms) {
            // system_state;
            init = uxr_millis();
        }
    } while (0);
}

/*******************
 * Setup and loop
 *******************/

void setup() {
    // microROS
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    p_msg_vl53l4cd_stamped = vl53l4cd_msgs__msg__Vl53l4cdStamped__create();
    // p_msg_ism330dhcx_stamped = ism330dhcx_msgs__msg__Ism330dhcxStamped__create();

    // IMU
    // setup_imu_sensor();
    // attachInterrupt(digitalPinToInterrupt(imu_sensor_info.interrupt1), imu_DRDY_interrupt_handler, RISING);
    // attachInterrupt(digitalPinToInterrupt(imu_sensor_info.interrupt1), imu_FIFO_Full_interrupt_handler, RISING);

    // ToF
    attachInterrupt(tof_sensors[0].interrupt_pin, tof0_interrupt_handler, FALLING);
    attachInterrupt(tof_sensors[1].interrupt_pin, tof1_interrupt_handler, FALLING);
    setup_tof_sensors();

    // LED
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    // read_imu_sensor_and_publish();
    // Check system state for connection status to micro_ros_agent
    switch (system_state) {
        case AGENT_WAIT:
            execute_every_n_ms(
                500, system_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : AGENT_WAIT);
            break;
        case AGENT_AVAILABLE:
            system_state = (true == create_rcl_entities()) ? AGENT_CONNECTED : AGENT_WAIT;

            if (system_state == AGENT_WAIT) {
                destroy_rcl_entities();
            }

            // RCSOFTCHECK(rmw_uros_sync_session(1000));
            break;
        case AGENT_CONNECTED:
            // Publish message
            execute_every_n_ms(
                10, system_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED);
            if (system_state == AGENT_CONNECTED) {
                // RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

                // Publish the message if it's available.
                RCSOFTCHECK(rmw_uros_sync_session(100));
                read_tof_sensors_and_publish();
                // read_imu_sensor_and_publish();
            }
            break;
        case AGENT_DISCONNECTED:
            destroy_rcl_entities();
            system_state = AGENT_WAIT;
            break;
        default:
            break;
    }

    // LED control
    if (system_state == AGENT_CONNECTED) {
        unsigned long now = millis();
        if (now - led_blink_start_time > 9900) {
            digitalWrite(LED_BUILTIN, HIGH);
        }
        if (now - led_blink_start_time > 10000) {
            digitalWrite(LED_BUILTIN, LOW);
            led_blink_start_time = now;
        }
    }
}
