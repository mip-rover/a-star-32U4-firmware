#include <AStar32U4.h>
#include <PololuRPiSlave.h>
#include <TimerThree.h>
#include <elapsedMillis.h>
#include <EnableInterrupt.h>

#define POWER_STATUS_PIN 18 
#define RPI_POWER_PIN 22

#define LEFT_MOTOR_ENCODER_PIN_A 15
#define LEFT_MOTOR_ENCODER_PIN_B 16
#define RIGHT_MOTOR_ENCODER_PIN_A 7
#define RIGHT_MOTOR_ENCODER_PIN_B 8
#define RPI_INTERRUPT_PIN 11
#define I2C_SLAVE_ADDRESS 20

struct motor_control_data
{
  uint8_t button_a;            // Address 3
  uint8_t button_b;
  uint8_t button_c;

  uint8_t cell_count;          // Address 21
  uint16_t battery_millivolts; // Address 24
  uint16_t low_voltage_cutoff; // Address 22

  uint32_t left_motor_count;   // Address 9
  uint32_t right_motor_count;  // Address 13
  uint16_t left_motor_speed;   // Address 17
  uint16_t right_motor_speed;  // Address 19

  int16_t left_motor;          // Address 4
  int16_t right_motor;         // Address 6

  bool clear_motor_counts;     // Address 8

  bool yellow;                 // Address 0
  bool green;                  // Address 1
  bool red;                    // Address 2
  
  bool play_notes;             // Address 27
  char notes[16];              // Address 28
};

const unsigned long speed_sample_rate = 20000;
const unsigned int cell_low_v = 3300;
const unsigned int cell_noload_low_v = 3700;
const unsigned int cell_noload_high_v = 4200;
const unsigned int battery_monitor_rate = 5000;

PololuRPiSlave<struct motor_control_data, 10> slave;

AStar32U4Buzzer buzzer;
AStar32U4ButtonA button_a;
AStar32U4ButtonB button_b;
AStar32U4ButtonC button_c;
AStar32U4Motors motors;

elapsedMillis last_battery_read;
bool load_notes = true;

unsigned short left_motor_count_a = 0;
unsigned short left_motor_count_b = 0;
unsigned short right_motor_count_a = 0;
unsigned short right_motor_count_b = 0;
unsigned long left_motor_count = 0;
unsigned long right_motor_count = 0;
unsigned short left_motor_speed = 0;
unsigned short right_motor_speed = 0;

static inline bool is_battery_power()
{
  return digitalRead(POWER_STATUS_PIN) == LOW;
}

static inline void enable_pi_power(bool on)
{
  digitalWrite(RPI_POWER_PIN, !on);
}

static inline void log(const String msg)
{
  if (usbPowerPresent)
    SerialUSB.println(msg);
}

void error()
{
  bool led_state = true;

  // Kill the motors */
  motors.setSpeeds(0, 0);

  // Turn off PI
  enable_pi_power(false);
  
  // Loop forevery
  while (true) {

    // Flash the red led
    ledYellow(led_state);
    led_state = !led_state;

    // Play siren
    buzzer.play("A8>>A8<<");
    while (buzzer.playCheck());

    // TODO Put CPU into deep sleep on min low voltage cut off
  }
}


void count_left_motor_a()
{
  ++left_motor_count_a;
}

void count_left_motor_b()
{
  ++left_motor_count_b;
}

void count_right_motor_a()
{
  ++right_motor_count_a;
}

void count_right_motor_b()
{
  ++right_motor_count_b;
}

void update_motor_counts()
{
  left_motor_speed = left_motor_count_a + left_motor_count_b;
  right_motor_speed = right_motor_count_a + right_motor_count_b;

  left_motor_count += left_motor_speed;
  right_motor_count += right_motor_speed;

  left_motor_count_a = 0;
  left_motor_count_b = 0;

  right_motor_count_a = 0;
  right_motor_count_b = 0;
 }

uint8_t count_cells()
{
  // Make we have at least a single cell LIPO
  unsigned short battery_voltage = readBatteryMillivoltsSV();
  if (battery_voltage < cell_noload_low_v)
    error();

  // Count the cells based on the input voltage
  for (uint8_t cells = 1; cells < 9; ++cells) {
    if(battery_voltage < cells * cell_noload_high_v)
      return cells;
  }

  // Uhmm, overvoltage, not sure why the board has not let out the magic smoke
  error();
}

void setup() 
{
  // Initialize the I2C slave
  slave.init(I2C_SLAVE_ADDRESS);

  // Initialize the I2C buffer
  slave.updateBuffer();
  slave.buffer.yellow = false;
  slave.buffer.green = false;
  slave.buffer.red = false;
  
  slave.buffer.button_a = 0;
  slave.buffer.button_b = 0;
  slave.buffer.button_c = 0;
  
  slave.buffer.left_motor = 0;
  slave.buffer.right_motor = 0;
  
  slave.buffer.clear_motor_counts = false;
  slave.buffer.left_motor_count = 0;
  slave.buffer.right_motor_count = 0;
  slave.buffer.left_motor_speed = 0;
  slave.buffer.right_motor_speed = 0;
  
  slave.buffer.cell_count = 0;
  slave.buffer.low_voltage_cutoff = 0;
  slave.buffer.battery_millivolts = 0;
  
  slave.buffer.play_notes = false;
  slave.finalizeWrites();

  // Let the PI run by default
  digitalWrite(RPI_POWER_PIN, LOW);

  // Setup the pins
  pinMode(LEFT_MOTOR_ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(LEFT_MOTOR_ENCODER_PIN_B, INPUT_PULLUP);
  pinMode(RIGHT_MOTOR_ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(RIGHT_MOTOR_ENCODER_PIN_B, INPUT_PULLUP);
  pinMode(POWER_STATUS_PIN, INPUT_PULLUP);
  pinMode(RPI_POWER_PIN, OUTPUT);

  // Enable the encoder interrupts
  enableInterrupt(LEFT_MOTOR_ENCODER_PIN_A, count_left_motor_a, CHANGE);
  enableInterrupt(LEFT_MOTOR_ENCODER_PIN_B, count_left_motor_b, CHANGE);
  enableInterrupt(RIGHT_MOTOR_ENCODER_PIN_A, count_right_motor_a, CHANGE);
  enableInterrupt(RIGHT_MOTOR_ENCODER_PIN_B, count_right_motor_b, CHANGE);

  // Enable speed sampling
  Timer3.initialize(speed_sample_rate);
  Timer3.attachInterrupt(update_motor_counts);

  // Initial the buzzer and play startup
  buzzer.playMode(PLAY_CHECK);
  buzzer.play("v15>>g16>>>c16>>g16>>>c16");
  while (buzzer.playCheck());

  // TODO Play cell count beeps
}

void loop() 
{
  // Get started update i2c buffer
  slave.updateBuffer();

  // Report current motor counts and speeds
  noInterrupts();
  slave.buffer.left_motor_count = left_motor_count;
  slave.buffer.right_motor_count = right_motor_count;
  slave.buffer.left_motor_speed = left_motor_speed;
  slave.buffer.right_motor_speed = right_motor_speed;
  if (slave.buffer.clear_motor_counts) {
    left_motor_count = 0;
    right_motor_count = 0;
    slave.buffer.clear_motor_counts = 0;
  }
  interrupts();

  // Track requested motor speeds
  motors.setSpeeds(slave.buffer.left_motor, slave.buffer.right_motor);

  // Track power source
  if (is_battery_power()) {
    if (slave.buffer.cell_count == 0) {
      slave.buffer.cell_count = count_cells();
      slave.buffer.low_voltage_cutoff = slave.buffer.cell_count * cell_noload_low_v;
      enable_pi_power(true);
    }
  } else {
    slave.buffer.cell_count = 0;
    slave.buffer.low_voltage_cutoff = 0;
    enable_pi_power(false);
  }

  // Monitor battery status
  if (slave.buffer.cell_count != 0 && last_battery_read > battery_monitor_rate) {
    slave.buffer.battery_millivolts = readBatteryMillivoltsSV();
    if (slave.buffer.battery_millivolts < slave.buffer.low_voltage_cutoff)
      error();
    last_battery_read = 0;
  }
 
  // Track leds
  ledYellow(slave.buffer.yellow);
  ledGreen(slave.buffer.green);
  ledRed(slave.buffer.red);

  // Report button presses
  slave.buffer.button_a += button_a.getSingleDebouncedPress();
  slave.buffer.button_b += button_b.getSingleDebouncedPress();
  slave.buffer.button_c += button_c.getSingleDebouncedPress();

  // Play sounds
  if (slave.buffer.play_notes) {
    if (load_notes)
      buzzer.play(slave.buffer.notes);
    slave.buffer.play_notes = buzzer.playCheck();
    load_notes = !slave.buffer.play_notes;
  }

  // All done, report any changes
  slave.finalizeWrites();
}
