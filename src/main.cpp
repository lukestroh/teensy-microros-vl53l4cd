#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>
#include <std_msgs/msg/string.h>

#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <vl53l4cd_msgs/msg/vl53l4cd.h>
#include <vl53l4cd_msgs/msg/vl53l4cd_stamped.h>


#define DEV_I2C_0 Wire
#define DEV_I2C_1 Wire1

#define SENSOR_0_ADDR 0x50
#define SENSOR_1_ADDR 0x52


typedef struct {
    VL53L4CD* psensor; // pointer to sensor object
    TwoWire* pwire;
    int addr;
    int id;     // sensor id
    int shutdown_pin;
    int interrupt_pin;
    uint8_t range_status;
    int16_t distance_mm;
} sensor_t;

VL53L4CD sensor0(&DEV_I2C_0, 21);
VL53L4CD sensor1(&DEV_I2C_1, 11);


// Sensors
sensor_t sensors[] = {
    {&sensor0, &DEV_I2C_0, SENSOR_0_ADDR, 0, 21, 20, 0, 0},
    {&sensor1, &DEV_I2C_1, SENSOR_1_ADDR, 1, 11, 12, 0, 0},
};
const uint8_t NUM_SENSORS = sizeof(sensors) / sizeof(sensors[0]);
volatile uint8_t interrupt_flags[] = {0,0};

// LED config
uint32_t led_blink_start_time = millis();

/* microROS setup */
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)){soft_error();}}

rcl_publisher_t publisher;
vl53l4cd_msgs__msg__Vl53l4cdStamped msg_vl53l4cd_stamped;
vl53l4cd_msgs__msg__Vl53l4cdStamped* p_msg_vl53l4cd_stamped;
// std_msgs__msg__String frame_id;
rosidl_runtime_c__String frame_id;
// rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
// rcl_timer_t timer;

// microROS system state
enum SystemState {
    AGENT_WAIT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} system_state;

/*******************
 * Sensors
*******************/

void sensor0_interrupt_handler() {
    interrupt_flags[0] = 1;
}

void sensor1_interrupt_handler() {
    interrupt_flags[1] = 1;
}

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

void setup_sensors() {
    VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;

    for (uint8_t i=0; i<NUM_SENSORS; ++i) {
        sensors[i].pwire->begin(); // begin the Wisre instance
        // delay(10);
        sensors[i].psensor->begin(); // configure the sensor with a power cycle.
        // delay(10);
        status = sensors[i].psensor->InitSensor(sensors[i].addr);
        if (status) {
            error_loop();
        }
        sensors[i].psensor->VL53L4CD_SetRangeTiming(50, 0); // highest timing budget (200) while avoiding low-power mode (0)
        sensors[i].psensor->VL53L4CD_StartTemperatureUpdate();
        sensors[i].psensor->VL53L4CD_StartRanging();

        pinMode(sensors[i].interrupt_pin, INPUT_PULLUP);
        delay(10);
    }

    

}

void read_sensors_and_publish() {
    uint8_t data_ready = 0;
    VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;
    VL53L4CD_Result_t results;

    for (uint8_t i=0; i<NUM_SENSORS; ++i) {
        if (interrupt_flags[i]) {
            interrupt_flags[i] = 0;
            status = sensors[i].psensor->VL53L4CD_CheckForDataReady(&data_ready);

            if ((!status) && (data_ready != 0)) {
                sensors[i].psensor->VL53L4CD_ClearInterrupt();
                sensors[i].psensor->VL53L4CD_GetResult(&results);

                // create message if results
                if (results.range_status == 0) {
                    p_msg_vl53l4cd_stamped->dev_id = sensors[i].id;
                    p_msg_vl53l4cd_stamped->status = results.range_status;
                    p_msg_vl53l4cd_stamped->distance = results.distance_mm;
                    p_msg_vl53l4cd_stamped->header.stamp.sec = static_cast<uint64_t>(rmw_uros_epoch_millis() / 1000);
                    rcl_publish(&publisher, p_msg_vl53l4cd_stamped, NULL);
                }
                // Serial.print(F("ID: "));
                // Serial.print(sensors[i].id);
                // Serial.print(F(", status: "));
                // Serial.print(results.range_status);
                // Serial.print(F(", dist: "));
                // Serial.println(results.distance_mm);
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
    const char* _namespace = "microROS";
    const char* _node_name = "teensy";
    RCCHECK(rclc_node_init_default(&node, _node_name, _namespace, &support));
  
    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(vl53l4cd_msgs, msg, Vl53l4cdStamped),
        "vl53l4cd/data"
    ));
    
    return true;
}

rcl_ret_t destroy_rcl_entities() {
    rcl_ret_t ret_status = RMW_RET_OK;
    /* Destroy the microROS-related entities */
    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  
    ret_status = rcl_publisher_fini(&publisher, &node);
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
    }
    while (0);
}

/*******************
 * Setup and loop
*******************/

void setup() {
    // delay(3000);
    Serial.begin(115200);
    set_microros_serial_transports(Serial);


    pinMode(LED_BUILTIN, OUTPUT);

    attachInterrupt(sensors[0].interrupt_pin, sensor0_interrupt_handler, FALLING);
    attachInterrupt(sensors[1].interrupt_pin, sensor1_interrupt_handler, FALLING);
    
    setup_sensors();
    
    p_msg_vl53l4cd_stamped = vl53l4cd_msgs__msg__Vl53l4cdStamped__create();
}

void loop() {

    // Check system state for connection status to micro_ros_agent
    switch (system_state) {
        case AGENT_WAIT:
            execute_every_n_ms(500, system_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : AGENT_WAIT);
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
            execute_every_n_ms(10, system_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED);
            if (system_state == AGENT_CONNECTED) {
                // RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

                // Publish the message if it's available.
                RCSOFTCHECK(rmw_uros_sync_session(100));
                read_sensors_and_publish();
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