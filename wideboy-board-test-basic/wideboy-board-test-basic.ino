#include <AStar32U4.h>
#include <EnableInterrupt.h>

const unsigned int update_rate = 1000;
const unsigned int max_motor_speed = 50;

PololuBuzzer buzzer;
AStar32U4Motors motors;
AStar32U4ButtonB button;

bool running = false;
int next_update = 0;

uint32_t rm_count = 0;
uint32_t lm_count = 0;

void rm_interrupt()
{
  ++rm_count;
}

void lm_interrupt()
{
  ++lm_count;  
}

void setup() 
{
  
  // Start playing a tone with frequency 440 Hz at maximum
  // volume (15) for 200 milliseconds.
  buzzer.playFrequency(440, 200, 15);
  while (buzzer.isPlaying());
  buzzer.playFrequency(220, 200, 15);
  while (buzzer.isPlaying());
  buzzer.playFrequency(880, 200, 15);
  while (buzzer.isPlaying());
  buzzer.playFrequency(440, 200, 15);
  while (buzzer.isPlaying());

  SerialUSB.begin(230400);

  pinMode(14, INPUT_PULLUP);
  pinMode(15, INPUT_PULLUP);
  pinMode(16, INPUT_PULLUP);
  pinMode(17, INPUT_PULLUP);
  
  enableInterrupt(14, lm_interrupt, CHANGE);
  enableInterrupt(15, lm_interrupt, CHANGE);
  enableInterrupt(16, rm_interrupt, CHANGE);
  enableInterrupt(17, rm_interrupt, CHANGE);

  ledYellow(running);

}

void loop() 
{

  if (button.getSingleDebouncedPress()) {

    int start_speed;
    int end_speed;
    int speed_step;
    
    if (running) {
      start_speed = max_motor_speed;
      end_speed = 0;
      speed_step = -1;
      running = false;
      buzzer.playFrequency(880, 300, 15);
      while (buzzer.isPlaying());
      buzzer.playFrequency(220, 200, 15);
      while (buzzer.isPlaying());
    } else {
      lm_count = 0;
      rm_count = 0;
      start_speed = 0;
      end_speed = max_motor_speed;
      speed_step = 1;
      running = true;
      buzzer.playFrequency(220, 300, 15);
      while (buzzer.isPlaying());
      buzzer.playFrequency(880, 200, 15);
      while (buzzer.isPlaying());
    }

    for (int speed = start_speed; speed != end_speed; speed += speed_step) {
      motors.setSpeeds(speed, speed);
      delay(2);
    }
  }
  
  if (millis() > next_update) {
    next_update = millis() + update_rate;
    
    int32_t right_motor_count = rm_count;
    int32_t left_motor_count = lm_count;
  
    SerialUSB.print("battery: ");
    SerialUSB.println(readBatteryMillivoltsSV());
    SerialUSB.print("right motor: ");
    SerialUSB.println(right_motor_count);
    SerialUSB.print("left motor: ");
    SerialUSB.println(left_motor_count);
  }

  ledYellow(running);
}
