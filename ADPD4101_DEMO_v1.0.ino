/*
ADPD-DEMO
Arduino Nano Every board
UART-to-I2C interface for ADPD4101 optical sensor board
*/



/*********************************************************************************
*          INCLUDES                          *
********* ************************************/

#include <math.h>
#include <Arduino.h>
#include <SerialCommands.h>
#include <Wire.h>

#include "adpd4101_app.h"


/*********************************************************************************
*          DEFINES                           *
*********************************************/

#define ADPD4101_NUM_CH 8  // we have this in the adpd4101_app_config.h as well, it would be nice to count up from the number of enabled slots and enabled channel2-s


/*********************************************************************************
*          DECLARATIONS                      *
*********************************************/
// SERIAL COMMAND-FUNCTIONS
char serial_command_buffer_[32];
void help(SerialCommands* sender);                      // Print available commands
void init(SerialCommands* sender);                      // init adpd4101
void measure(SerialCommands* sender);                   // measure single
void continous(SerialCommands* sender);                 // continous measurement
void stop(SerialCommands* sender);                      // Function to stop continous measurement
void register_read(SerialCommands* sender);             //Function to read register
void register_write(SerialCommands* sender);            //Function to write register


// MISC FUNCTIONS
void check_for_new_data();
void fastblink();
void slowblink();

// GLOBAL VARIABLES
double some_parameter = 0;
bool cont_mode = false;

struct adpd4101_app_dev* adpd4101_app;



/*********************************************************************************
*          SERIAL COMMANDS                   *
*********************************************/

SerialCommands serial_commands_(&Serial, serial_command_buffer_, sizeof(serial_command_buffer_), "\r\n", " ");
SerialCommand help_("HELP", help);                                          // Define "HELP" as the serial command
SerialCommand init_("INIT", init);                                          // Define "INIT" as the serial command
SerialCommand measure_("MEAS", measure);                                    // Define "MEAS" as the serial command
SerialCommand continous_("CONT", continous);                                // Define "CONT" as the serial command
SerialCommand stop_("STOP", stop);                                          // Define "STOP" as the serial command
SerialCommand register_read_("REGR", register_read);                        // Define "REGR" as the serial comand
SerialCommand register_write_("REGW", register_write);                      //Define "REGW" as the serial comand



/*********************************************************************************
*          SETUP                             *
*********************************************/
void setup() {
  Serial.begin(57600);
  while (!Serial)
    ;  //wait for Serial to be available

  // ADD SERIAL COMMANDS
  serial_commands_.SetDefaultHandler(cmd_unrecognized);
  serial_commands_.AddCommand(&help_);
  serial_commands_.AddCommand(&init_);
  serial_commands_.AddCommand(&measure_);
  serial_commands_.AddCommand(&continous_);
  serial_commands_.AddCommand(&stop_);
  serial_commands_.AddCommand(&register_read_);
  serial_commands_.AddCommand(&register_write_);


  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW

  Serial.println(F("Arduino SETUP: READY!"));  // Setup done, print to serial

}



/*********************************************************************************
*          RUNS FOREVER                      *
*********************************************/
void loop() {
  // put your main code here, to run repeatedly:
  serial_commands_.ReadSerial();

  if (cont_mode) {
    check_for_new_data();
  }
}




/*********************************************************************************
*          FUNCTIONS                         *
*********************************************/
// COMMAND: UNRECOGNIZED
void cmd_unrecognized(SerialCommands* sender, const char* cmd) {  // Default handler
  sender->GetSerial()->print(F("Unrecognized command: ''"));
  sender->GetSerial()->print(cmd);
  sender->GetSerial()->println(F("'', try ''HELP''!"));
}

// COMMAND: HELP
void help(SerialCommands* sender) {  // Display help
  sender->GetSerial()->println(F("Commands available:"));
  sender->GetSerial()->println(F("Initialize ADPD4101: ''INIT''"));
  sender->GetSerial()->println(F("Single measurement: ''MEAS''"));
  sender->GetSerial()->println(F("Start continous measurement: ''CONT''"));
  sender->GetSerial()->println(F("Stop continous measurement: ''STOP''"));
  sender->GetSerial()->println(F("Register read from hex address addr: ''REGR addr''"));
  sender->GetSerial()->println(F("Register write to hex addr the hex data with hex mask: ''REGW addr data mask''"));  
}

// COMMAND: INIT
void init(SerialCommands* sender) {  // initialize adpd4101
  int32_t ret = 0;
  
  stop(sender);
  sender->GetSerial()->println(F("init adpd4101..."));
  ret = adpd4101_app_init(&adpd4101_app);
  if (ret == 0) {
    sender->GetSerial()->println(F("init OK"));
    slowblink();  // LED blinks one - OK
    return;
  } else {
    sender->GetSerial()->println(F("init FAILED"));
    fastblink();  // LED blinks fast - error
    return;
  }
}


// COMMAND: MEAS
void measure(SerialCommands* sender) {
  int32_t ret = 0;
  uint32_t buff[ADPD4101_NUM_CH];
  uint8_t i = 0;

  stop(sender);
  
  for (i = 0; i < ADPD4101_NUM_CH; i++) {
    buff[i] = 0;
  }
  
  ret = adpd4101_app_read_samples(buff, 1);  // nbr_of_reads can be dynamic! (buff needs to be that times longer)
  for (i = 0; i < ADPD4101_NUM_CH; i++) {
    sender->GetSerial()->print(buff[i]);
    if (i < ADPD4101_NUM_CH) {
      sender->GetSerial()->print("\t");
    }
  }
  sender->GetSerial()->print(F("\n"));

  return;
}



// COMMAND: CONT
void continous(SerialCommands* sender) {
  int32_t ret;
  if (!cont_mode) {
    cont_mode = true;
    ret = adpd4101_app_start_continous();
    if (ret != 0)
      sender->GetSerial()->println(F("error: measurement not starting"));
  }
  return;
}


// COMMAND: STOP
void stop(SerialCommands* sender) {
  int32_t ret;
  if (cont_mode) {
    cont_mode = false;
    ret = adpd4101_app_stop_continous();
    if (ret != 0)
      sender->GetSerial()->println(F("error: measurement not stopping"));
  }
  return;
}


//COMMAND: REGR addr
void register_read(SerialCommands* sender) {
  int32_t ret = 0;
  sender->GetSerial()->println(F("reg read begin"));
  
  char* reg_name = sender->Next();
  if (reg_name == NULL) {
    sender->GetSerial()->println(F("ERROR! PARAMETER NOT SPECIFIED"));
    return;
  }

  ret = adpd4101_app_register_read(reg_name);
  if (ret == 0) {
    sender->GetSerial()->println(F("reg read OK"));
    return;
  } 
  else {
    sender->GetSerial()->println(F("reg read  FAILED"));
    return;
  }
  return;
}



//COMMAND: REGW addr data mask
void register_write(SerialCommands* sender){
  int32_t ret = 0;
  //sender->GetSerial()->println(F("reg write begin"));

  char* reg_name = sender->Next();
  if (reg_name == NULL) {
    sender->GetSerial()->println(F("ERROR! PARAMETER NOT SPECIFIED"));
    return;
  }
  char* reg_val = sender->Next();
  if (reg_val == NULL) {
    sender->GetSerial()->println(F("ERROR! PARAMETER NOT SPECIFIED"));
    return;
  }
  char* mask=sender->Next();
  if (mask == NULL) {
    sender->GetSerial()->println(F("ERROR! PARAMETER NOT SPECIFIED"));
    return;
  }
  
  ret = adpd4101_app_register_write(reg_name, reg_val, mask);
  if (ret == 0) {
    sender->GetSerial()->println(F("reg write OK"));
    return;
  } 
  else {
    sender->GetSerial()->println(F("reg write  FAILED"));
    return;
  }
 return;
}




void check_for_new_data() {
  int32_t ret = 0;
  uint32_t buff[ADPD4101_NUM_CH];
  uint8_t i = 0;
  bool data_ready = false;

  for (i = 0; i < ADPD4101_NUM_CH; i++) {
    buff[i] = 0;
  }

  ret = adpd4101_app_update_data((uint32_t*)buff, &data_ready);
  if (ret != 0) {
    Serial.print(F("error"));
    return;
  }
  
  if (data_ready) {
    for (i = 0; i < ADPD4101_NUM_CH; i++) {
      Serial.print(buff[i]);
      if (i < ADPD4101_NUM_CH-1) {
        Serial.print("\t");
      }
    }
    Serial.print(F("\n"));

    return;
  }
}


void fastblink() {
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(250);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(250);                       // wait for a second
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(250);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(250);                       // wait for a second
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(250);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
}

void slowblink() {
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(500);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
}
