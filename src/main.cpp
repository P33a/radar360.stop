#include <Arduino.h>
#include <EEPROM.h>
#include "proj_types.h"
#include "dchannels.h"
#include "ArduinoNvs.h"

#include <WebServer.h>
#include <AutoConnect.h>

#include "staging_buffer.h"

#include <ArduinoOTA.h>

#include "PCF8574.h"
#include <Wire.h>

// adjust addresses if needed
PCF8574 PCF_IN(0x24);  // add switches to lines  (used as input)

PCF8574 PCF_OUT(0x20);  // add leds to lines      (used as output)

uint8_t pcf_out_data, pcf_in_data;

WebServer Server;
AutoConnect Portal(Server);
AutoConnectConfig config;

const int led_pin = 2;
uint8_t led_state = 0;  

int last_micros = 0;        // last control time
int interval_us;            // Control period in microseconds

const int end_pin = 13;   // D7

const int dir_pin = 26;    // D3
const int step_pin = 27;   // D4
const int enable_pin = 23; // MOSI

const int heartbeat_pin = led_pin; // D9
const int person_pin = 25; // D2

const int PIR0_pin = 34; // A2
const int PIR1_pin = 35; // A3
const int PIR2_pin = 15; // A4
const int PIR3_pin = 39; // A1

dchannels_t serial_channels, udp_channels;

#define NUM_PARS 16

float Pars[NUM_PARS];
uint8_t send_par_index, udp_par_index;

#define SAVE_BUFFER_SIZE (4 + NUM_PARS * sizeof(float))

uint8_t save_buffer[SAVE_BUFFER_SIZE];
int save_par_byte;
uint8_t save_requested;

staging_buffer_t staging_buffer;
IPAddress SendIP(192, 168, 1, 172);
int wifi_on, prev_wifi_on, udp_on;

WiFiUDP Udp;
unsigned int localUdpPort = 4224;  // local port to listen on

#define UDP_MAX_SIZE 512
uint8_t UdpInPacket[UDP_MAX_SIZE];  // buffer for incoming packets
uint8_t UdpOutPacket[UDP_MAX_SIZE];  // buffer for outgoing packets
int UdpBufferSize = UDP_MAX_SIZE;

char  replyPacekt[] = "Hi there! Got the message :-)";  // a reply string to send back

#define sweep_speed Pars[0]
#define step_steps Pars[1]

  
int PIR[4];
int end_switch;

stepper_t motor;

hw_timer_t * timer = NULL;

fsm_t fsm;

int pir_person;

void save_prepare(void);
void save_pars_to_eeprom(void);
uint8_t read_pars_from_eeprom(void);
void fill_sane_pars(void);

void set_interval(uint32_t new_interval_us)
{
  interval_us = new_interval_us;
  //dt = interval_us * 1e-6;
}


uint32_t crc32c(uint8_t* data, uint8_t num_bytes, uint32_t chain)
{
  uint32_t b, mask;
  uint32_t crc = 0xFFFFFFFF ^ chain;
  uint8_t i;
  int8_t shift;

  for(i = 0 ; i < num_bytes ; i++ ) {
    b = data[i];
    crc = crc ^ b;

    for(shift = 7; shift >= 0; shift--) {
      mask = -(crc & 1);
      crc = (crc >> 1) ^ (0xEDB88320 & mask);
    }
  }
  return ~crc;
}

void IRAM_ATTR onTimer(void) 
{
  motor.hi_low++;
  if (motor.hi_low & 1) {
    digitalWrite(step_pin, 1);        // This is when the driver sees a step
    motor.steps += 1 - 2 * motor.dir;
  } else {
    digitalWrite(dir_pin, motor.dir); // Only change dir whe step_pin is low
    digitalWrite(step_pin, 0);        
  }
}

void set_motor_speed(float tps)
{
  motor.tps = tps;

  if (tps == 0) {
    timerAlarmDisable(timer);
    motor.enable = 0;

  } else {
    motor.enable = 1;
    motor.period =  1e6 / fabs(tps * 32.0 * 400.0 * 2.0);
    timerAlarmWrite(timer, motor.period, true);
    if (!timerAlarmEnabled(timer)) {
      timerAlarmEnable(timer);
    }
    if (tps > 0) {
      motor.dir = 0;  
    } else {
      motor.dir = 1;  
    }
  }
  digitalWrite(enable_pin, !motor.enable);  
}

int flag = 0;
int flagStop = 0;

void fsm_t::progress(void)
{
  tis = millis() - tes;

  if (state == ps_init) {
    set_state(ps_sync);

  } else if (state == ps_sync && (end_switch || tis > 3000)) {  // Timeout when homing
    //set_state(ps_idle);
    motor.steps = 0;
    set_state(ps_sweep);

  } else if (state == ps_sync && end_switch) {
    motor.steps = 0;
    set_state(ps_sweep);

  } else if (state == ps_sweep && motor.steps > 640*4.7) {
    set_state(ps_stop);

  //} else if (state == ps_stop && tis > 800) {
  } else if (state == ps_stop && (flag == 1 || tis > 2000) && (!flagStop)) {
    flag = 0;
    motor.steps -= 640*4.7; 
    set_state(ps_sweep);
  
  } else if (state == ps_stop && flagStop) { 
    set_state(ps_stop);

  } 

}

bool sendStateflag = false;

void fsm_t::set_state(fsm_state_t new_state)
{
  if (new_state != state) {
    // Enter here if there was a state change
    prev_state = state;

    // Reset Time Entering State (tes)
    tes = millis();

    // Things to do when entering a new state
    if (new_state == ps_sync) {
      set_motor_speed(sweep_speed);

    } else if (new_state == ps_sweep) {
      set_motor_speed(sweep_speed);

    } else if (new_state == ps_stop) {
      set_motor_speed(0);
      sendStateflag = true;

    } else if (new_state == ps_idle) {
      set_motor_speed(0);
    }

    state = new_state;
  }
}


void fsm_t::act(void)
{
  switch (state) {
    
    case ps_init:
    break;
        
    case ps_idle:
    break;
    
    default:
    break;
  }
}

void process_serial_packet(char channel, char sub_channel, uint32_t value, dchannels_t& obj)
{
  byte i;

  if (channel == 'S')  {
    if (sub_channel == 'v') {
      save_prepare();
      save_pars_to_eeprom();
    }
    if (sub_channel == 's') {
      fsm.set_state((fsm_state_t)value);
    }

  } else if (channel == 'G')  {
    i = sub_channel - 'i';
    if (i < 16 && i >= 0) {
      Pars[i] = *((float *) &value);
    }  

  } else if (channel == 'I')  {
    set_interval(value);

  } else if (channel == 'N')  { // Ping
    obj.send(channel, sub_channel, value + 1);
    Serial.println(value + 1);
  }

}


void serial_write(uint8_t b)
{
  Serial.write(b);
}

void handle_UDP_buffer(uint8_t* buf)
{
  Udp.beginPacket(SendIP, 4210);
  Udp.write((uint8_t*)buf, staging_buffer.get_size());
  Udp.endPacket();
  staging_buffer.empty();
}

void udp_write(uint8_t b)
{
  staging_buffer.sendByte(b);
}


void on_wifi_on_event(void)
{
  Serial.println("WiFi connected: " + WiFi.localIP().toString());

  Udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();    
}

uint8_t last = 0;

void setup() 
{
  last = millis();

  pinMode(led_pin, OUTPUT);      
  digitalWrite(led_pin, 0);  

  pinMode(end_pin, INPUT_PULLUP);      
  
  pinMode(step_pin, OUTPUT);      
  digitalWrite(step_pin, 0);  

  pinMode(dir_pin, OUTPUT);      
  digitalWrite(dir_pin, 0);  
  
  pinMode(enable_pin, OUTPUT);      
  digitalWrite(enable_pin, 0);  
  
  pinMode(PIR0_pin, INPUT_PULLUP);      
  pinMode(PIR1_pin, INPUT_PULLUP);      
  pinMode(PIR2_pin, INPUT_PULLUP);     
  pinMode(PIR3_pin, INPUT_PULLUP);     
  
  pinMode(person_pin, OUTPUT);      
  digitalWrite(person_pin, 0);  

  PCF_IN.begin();
  PCF_OUT.begin();

  NVS.begin();
  if (!read_pars_from_eeprom()) {
    fill_sane_pars();
  }
  
  // Initilize Stepper Timer
  timer = timerBegin(3, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 500, true);
  timerAlarmEnable(timer);

  set_motor_speed(0);

  //set_interval(25000UL);
  set_interval(66000UL);


  // initialize serial communication:
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 19, 18);
  //Serial.begin(230400);
  
  serial_channels.init(process_serial_packet, serial_write);

  udp_channels.init(process_serial_packet, udp_write);
  staging_buffer.init(UdpOutPacket, handle_UDP_buffer, UdpBufferSize);

  fsm.state = ps_init;
  last_micros = micros();
  Serial.println("Init Done");

  config.portalTimeout = 10000;  // It will time out in x seconds
  config.retainPortal = true;
  Portal.config(config);

  wifi_on = 0;
  prev_wifi_on = 0;
  udp_on = 0;

  if (Portal.begin()) {
    wifi_on = 1;
    on_wifi_on_event();
  }  
}

uint8_t text_debug = 0;
uint8_t thermalOn = 0;
uint8_t rgbOn = 0;

void loop() 
{
  byte b; 
  //static int decimate_led = 0;
  int act_micros, delta;

  prev_wifi_on = wifi_on ;
  wifi_on = WiFi.status() == WL_CONNECTED;

  if (wifi_on && !prev_wifi_on) {
    prev_wifi_on = 1;
    on_wifi_on_event();
  }  

  if (wifi_on) {  // Code for the connected state
    ArduinoOTA.handle();

    int packetSize = Udp.parsePacket();
    if (packetSize) {
      int i;
      udp_on = 1;
      // receive incoming UDP packets
      // and forward it to the serial port

      //Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
      int len = Udp.read(UdpInPacket, UdpBufferSize - 1);
      if (len > 0) {
        UdpInPacket[len] = 0;
      }
      //Serial.printf("UDP packet contents (as string): %s\n", UdpInPacket);

      for (i = 0; i < len; i++) {
        udp_channels.StateMachine(UdpInPacket[i]);
        //Serial.write(UdpInPacket[i]);
      }
    }    

  } else {
    // Some sketch code for not connected scene is here.
  }
  Portal.handleClient();


  if (Serial.available()) {
    b = Serial.read();
    if (b == '!') text_debug = 0;
    if (b == '#') text_debug = 1;
    serial_channels.StateMachine(b);
  } 

  pcf_in_data = PCF_IN.read8();

  if (pcf_in_data & 0b00000001) {
    flagStop = 1;
    fsm.set_state(ps_stop);
  } 
  if (pcf_in_data & 0b00000010) {
    flagStop = 0;
  } 
  if (pcf_in_data & 0b00000011) {
    Serial2.write("5");
  } 

  if (Serial2.available()) {
    Serial.print("Message: ");
    int msg = Serial2.read();
    char msgS = msg;
    Serial.println(msgS);
    if (msg == 49) {
      //fsm.set_state(ps_sweep);
      flag = 1;
    } 
    if (msg == 50) {
      flagStop = 1;
      fsm.set_state(ps_stop);
    } 
    else if (msg == 51) {
      flagStop = 0;
    } 
    if (msg == 52) {
      thermalOn = 1;
    }
    if (msg == 53) {
      rgbOn = 1;
    }
  }

  if (sendStateflag) {
    Serial2.write("2");
    sendStateflag = false;
    Serial.println(fsm.state);
  }

  /*if (millis() - last > 1000) {
    Serial2.write("2");
    last += 1000;
    Serial.println("Check");
  }*/

  // Read end switch
  end_switch = !digitalRead(end_pin);

  // Do the state machine
  fsm.progress();
  fsm.act();
  
  act_micros = micros();
  delta = act_micros - last_micros;
  if (delta >= interval_us) {
    last_micros = act_micros;  
    //Serial.print(step);

    // Read PIRs
    PIR[0] = digitalRead(PIR0_pin);
    PIR[1] = digitalRead(PIR1_pin);
    PIR[2] = digitalRead(PIR2_pin);
    PIR[3] = digitalRead(PIR3_pin);
    pir_person = PIR[0] | PIR[1] | PIR[2] | PIR[3];
    digitalWrite(person_pin, pir_person);  

    long start_time = millis();
    long time_taken = millis() - start_time;

    digitalWrite(led_pin, led_state);

    //pcf_in_data = PCF_IN.read8();
    //pcf_out_data = pcf_in_data;
    //PCF_OUT.write8(pcf_out_data);
    
    if (text_debug) {
      Serial.print("S: ");
      Serial.print(fsm.state);

      Serial.print(" E: ");
      Serial.print(end_switch);

      Serial.print(" P[");
      Serial.print(PIR[0]);
      Serial.print(PIR[1]);
      Serial.print(PIR[2]);
      Serial.print(PIR[3]);
      Serial.print("] ");

      Serial.print(" Steps: ");
      Serial.print(motor.steps);

      Serial.print(" LED: ");
      Serial.print(led_state);

      Serial.print(" ");
    }

    Serial.printf("%s, Port %d ", WiFi.localIP().toString().c_str(), localUdpPort);

    // Send motor state
    serial_channels.send('S', 's', motor.tps * 1000, fsm.state); 
    serial_channels.send('S', 'p', time_taken, PIR[0] + 2 * PIR[1] + 4 * PIR[2] + 8 * end_switch + 16 * led_state + 32 * PIR[3]); 

    // send Pars Table sequentially (four for each loop)
    for (int i = 0; i < 4; i++) {
      serial_channels.sendFloat('G', 'i' + send_par_index, Pars[send_par_index]);
      send_par_index++;
      if (send_par_index >= NUM_PARS) send_par_index = 0;
    }
      
    serial_channels.send('P','o', delta); 

    Serial.println();    

    /*decimate_led++;
    if (decimate_led > 2) {
      // if the heartbeat LED is off turn it on and vice-versa:
      led_state = !led_state;
      //digitalWrite(led_pin, led_state);
      //Serial.print(led_state);
      decimate_led = 0;
    }*/

    if (WiFi.status() == WL_CONNECTED) {
      //udp_channels.sendFloat('T', 't', ambient_temperature);
      //udp_channels.sendFloat('T', 'm', heat.max_temp);
      // Send motor state
      udp_channels.send('S', 's', motor.tps * 1000, fsm.state); 
      udp_channels.send('S', 'p', time_taken, PIR[0] + 2 * PIR[1] + 4 * PIR[2] + 8 * end_switch + 16 * led_state + 32 * PIR[3]); 
    
      // send Pars Table sequentially (four for each loop)
      for (int i = 0; i < 4; i++) {
        udp_channels.sendFloat('G', 'i' + udp_par_index, Pars[udp_par_index]);
        udp_par_index++;
        if (udp_par_index >= NUM_PARS) udp_par_index = 0;
      }
      
      udp_channels.send('P','o', delta); 

      if (!staging_buffer.is_empty() && udp_on) {
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write((uint8_t*)staging_buffer.get_buffer(), staging_buffer.get_size());
        //Serial.print("Sent="); Serial.println(Udp.endPacket());
        Udp.endPacket();
        staging_buffer.empty();
       }    
    }
  }

  if (thermalOn) {
    pcf_out_data = 0b00000001;
  }
  if (rgbOn) {
    pcf_out_data = 0b00000010;
  }
  if (pir_person) {
    pcf_out_data = 0b00000100;    
  }
  if (rgbOn && thermalOn) {
    pcf_out_data = 0b00000011;    
  }
  if (rgbOn && pir_person) {
    pcf_out_data = 0b00000110;    
  }
  if (thermalOn && pir_person) {
    pcf_out_data = 0b00000101;    
  }
  if (thermalOn && pir_person && rgbOn) {
    pcf_out_data = 0b00000111;    
  }

  if (rgbOn || pir_person || thermalOn) {
    PCF_OUT.write8(pcf_out_data);
    rgbOn = 0;
    thermalOn = 0;
  }
}



// Incremente this value each time the meaning of any value in Pars[] changes
// When reading old Pars in EEPROM it will consider the stored values inavlid and restore the default ones
#define CRC32HEADER 0

// Non blocking EEPROM save for the Pars array

void save_prepare(void)
{
  int i;

  // Lets see the array of floats as an array of bytes
  byte* p = (byte*)Pars;

  // Fill the buffer (reserve 4 bytes for the crc32 check)
  for (i = 4; i < SAVE_BUFFER_SIZE; i++) {
    save_buffer[i] = *p;
    p++;
  }

  // Calculate the crc32 check for the buffer 
  uint32_t crc = crc32c(&(save_buffer[4]), SAVE_BUFFER_SIZE - 4, CRC32HEADER);
  // and store it on the first four bytes
  *((uint32_t*)save_buffer) = crc;

  save_par_byte = 0;
  save_requested = 1;
}

// save_buffer starts with a crc32 of the following data
// 4 * 16 bytes of data (the memory image of 16 floats from Pars[])
void save_pars_to_eeprom(void)
{
  if (save_requested) {
    NVS.setBlob("pars", save_buffer, sizeof(save_buffer));
    NVS.commit();
    save_requested = 0;
  }
}
/*
void save_pars_to_eeprom(void)
{
  // Never wait for a write
  if (!eeprom_is_ready()) return;

  // Save one byte if necessary
  eeprom_update_byte((uint8_t*)save_par_byte, save_buffer[save_par_byte]);

  // Next time we save the next byte
  save_par_byte++;
  if (save_par_byte >= (int)SAVE_BUFFER_SIZE) {
    save_par_byte = 0; 
    save_requested = 0;
  }
}*/

uint8_t read_pars_from_eeprom(void)
{
  //eeprom_read_block((void*)save_buffer, 0, SAVE_BUFFER_SIZE);
  NVS.getBlob("pars", save_buffer, sizeof(save_buffer));
  // Calculate the crc32 check for the buffer 
  uint32_t crc = crc32c(&(save_buffer[4]), SAVE_BUFFER_SIZE - 4, CRC32HEADER);
  if (*((uint32_t*)save_buffer) == crc ) {
    byte* p = (byte*)Pars;

    // Fill the buffer 
    for (int i = 4; i < SAVE_BUFFER_SIZE; i++) {
      *p = save_buffer[i];
      p++;
    }
    return 1;
  } else {
    return 0; // The parameter were invalid
  }
}

void fill_sane_pars(void)
{
  Pars[0] = 1;  // Sweep speed
  Pars[1] = 640*4.7; // step_steps
}
