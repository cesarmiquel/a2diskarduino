#include <SdFat.h>

//
// To connect this to the Apple //c you need to connect:
//
// - pin 1 of the INTERNAL DISK J8 connector to the GND pin of the Arduino Nano
// - pin 2 (PH0) of the INTERNAL DISK J8 connector to A0 of the Arduino Nano
// - pin 4 (PH1) of the INTERNAL DISK J8 connector to A1 of the Arduino Nano
// - pin 6 (PH2) of the INTERNAL DISK J8 connector to A2 of the Arduino Nano
// - pin 8 (PH3) of the INTERNAL DISK J8 connector to A3 of the Arduino Nano
// - pin 16 (RDDATA) of the INTERNAL DISK J8 connector to pin D6 of the Arduino Nano
//
// Connect the Arduino to a USB to power it up and let the turn on the Apple //c.
//

#define SD_CHIPSELECT 10


// Apple 2c connection - J8
#define A2_PH0        14 // A0
#define A2_PH1        15 // A1
#define A2_PH2        16 // A2
#define A2_PH3        17 // A3
#define A2_RDDATA     6  // D6

#define KEY_PIN       A6 // A5

#define SECTOR_SIZE   416
#define NUM_SECTORS   16
#define NUM_TRACKS    35

#define KEY_CODE_UP     1
#define KEY_CODE_ENTER  2
#define KEY_CODE_DOWN   3

//#define DEBUG         1
#define DISPLAY_ENABLED 1


#ifdef DISPLAY_ENABLED

#include "SSD1306_minimal.h"

SSD1306_Mini ssd1306;
#endif


#define MAX_WAIT_MS     50

// Size of our buffer to transfer data to Apple //c.
#define BUFFER_SIZE       SECTOR_SIZE + 1

// sector buffer
uint8_t track_buffer[BUFFER_SIZE];

volatile int intcount = 0;
volatile int motor_active = 0;
volatile unsigned long last_phase_changed = 0; // in ms (millisecond)

uint8_t currentSector = 0;

// Display
#define MAX_FILENAME_SIZE  32
uint8_t g_current_line = 0;
char  g_current_image[MAX_FILENAME_SIZE];

// For now one image
SdFat32 SD;
FatFile sdf;

// Declaration of code which sends data to the Drive
extern "C" unsigned char StreamData(unsigned char*);

/*
 * ------------------------------------------------------ 
 * Code from lib/device/iwm/disk2.cpp of fujinet-firmware
 * ------------------------------------------------------
 */

volatile int track_pos;
volatile int old_pos;
volatile uint8_t oldphases;
volatile uint8_t newphases;

// To understand this code see this:
// https://github.com/FujiNetWIFI/fujinet-firmware/commit/ae26e9955e5b4201cbc71e4da48dae57deb57eff
//const PROGMEM int8_t phase2seq[16] = {-1, 0, 2, 1, 4, -1, 3, -1, 6, 7, -1, -1, 5, -1, -1, -1};
const PROGMEM int8_t seq2steps[8] = {0, 1, 2, 3, 0, -3, -2, -1};

const int8_t phase2seq[16] = {-1, 0, 2, 1, 4, -1, 3, -1, 6, 7, -1, -1, 5, -1, -1, -1};
//const int8_t seq2steps[8] = {0, 1, 2, 3, 0, -3, -2, -1};

#define MAX_TRACKS 160

bool phases_valid(uint8_t l_phases)
{
  return (phase2seq[l_phases] != -1);
  //return (pgm_read_byte(phase2seq + l_phases) != -1);
}

bool move_head()
{
  int8_t delta = 0;

  newphases = (PINC & 0x0f); // could access through IWM instead
  
  if (phases_valid(newphases))
  {
    int8_t idx = (phase2seq[newphases] - phase2seq[oldphases] + 8) % 8;
    //int8_t idx = (pgm_read_byte(phase2seq + newphases) - pgm_read_byte(phase2seq + oldphases) + 8) % 8;
    //delta = seq2steps[idx];
    delta = pgm_read_byte(seq2steps + idx);

    //phases_lut[oldphases][newphases];
    old_pos = track_pos;
    track_pos += delta;
    if (track_pos < 0)
    {
      track_pos = 0;
    }
    else if (track_pos > MAX_TRACKS-1)
    {
      track_pos = MAX_TRACKS-1;
    }
    oldphases = newphases;
  }
  return (delta != 0);
}
// -------------------------------------------------------------

// Interrupt service routine when phases change
ISR(PCINT1_vect)
{
  static uint8_t prev_phases = (PINC & 0x0f);

  uint8_t cur_phases = (PINC & 0x0f);
  bool ph1_changed = ((cur_phases & 0x02) != (prev_phases & 0x02));

  intcount++;

  // Skip edge case. From lib/bus/iwm/iwm_ll.cpp:
  // add extra condition here to stop edge case where on softsp, the disk is stepping inadvertantly when SP bus is
  // disabled. PH1 gets set low first, then PH3 follows a very short time after. We look for the interrupt on PH1 (33)
  // and then PH1 = 0 (going low) and PH3 = 1 (still high)
  if ( ph1_changed && cur_phases == 0b1000 ) {
    return;
  }

  // Probamos el codigo de fujinet
  if (move_head()) {
    motor_active = 1;
    last_phase_changed = millis();
  }
  return;
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(500000);

  Serial.print(F("\r\n>>>>>> Setting up USART...\r\n"));

#ifdef DISPLAY_ENABLED
  // put your setup code here, to run once:
  ssd1306.init(SlaveAddress);
  ssd1306.clear();
  ssd1306.cursorTo(0, 0);
  ssd1306.printString("Apple //c DISK");
#endif

  pinMode(A2_PH0, INPUT);
  pinMode(A2_PH1, INPUT);
  pinMode(A2_PH2, INPUT);
  pinMode(A2_PH3, INPUT);
  pinMode(A2_RDDATA, OUTPUT);

  // antes PCICR  = B00000010;
  PCICR |= (1 << PCINT1);
  PCMSK1 = B00001111;

  intcount = 0;
  motor_active = 0;

  // fujinet firmware initialization
  track_pos = 80;
  old_pos = 0;
  oldphases = 0;

  // Open SD card
  if (!SD.begin(SD_CHIPSELECT)) {
    //Serial.print("\r\nError init card\r\n");
  } else {

    refreshDisplay();

    // Open image boot.txt to see what image to use.
    //openImage("DISK01.NIB");
    File boot = SD.open("boot.txt");
    if (boot) {
      boot.read(g_current_image, MAX_FILENAME_SIZE);
      boot.close();
      openImage(g_current_image);
      Serial.print(F("Booting from ["));
      Serial.print(g_current_image);
      Serial.print(F("]\r\n"));
    }
  }

  Serial.flush();
}


void loop()
{
  static int prev_int_count = 0;

  static uint8_t cur_key = 0;
  static uint8_t prev_key = 0;

#ifdef DEBUG
  char msg[50];
#endif

  while (1) {

    cur_key = getCurrentKey();
    if (cur_key != prev_key) {
      if (cur_key > 0) {
        handleKeyPressed(cur_key);
      }
      prev_key = cur_key;
    }

#ifdef DEBUG
    if (prev_int_count != intcount) {
      prev_int_count = intcount;
      sprintf(msg, "Processed %d interrupts. motor_active %d\r\n", intcount, motor_active);
      Serial.print(msg);
      sprintf(msg, "FUJI: TRK: %d (%d). PH: %02x PREV PH: %02x\r\n", (track_pos+3) >> 2, track_pos, newphases, oldphases);
      Serial.print(msg);
    }
#endif
  
    // Stream sector from current track
    streamTrack();
  }
}

// stream current sector
void streamTrack()
{
  volatile unsigned long now;
  static uint8_t currentTrack = 0;
 
  if (sdf == 0) {
    // Can't do anything because disk is broken. Send message and
    // abort
    Serial.print("Can not read disk image.\r\n");
    delayMicroseconds(500000);
    return;
  }

  // If we are moving the track head no need to send anything
  now = millis();
  if (motor_active && ((now - last_phase_changed) < MAX_WAIT_MS)) {
    return;
  }

  motor_active = 0;

  // on one entry we read data from the SD on the next we stream it
  readSector();

  // Send stream.
  streamSector();

  // Move head to next sector
  currentSector++;
  if (currentSector >= NUM_SECTORS) {
    currentSector = 0;
  }

  // If track changed update status line
  if (((track_pos+3) >> 2) == currentTrack) {
    return;
  }
  currentTrack = ((track_pos+3)>>2);
  updateStatusLine(g_current_image);
}

void readSector()
{
  //sdf.seekSet( uint32_t ((track_pos+3) >> 2) * NUM_SECTORS * SECTOR_SIZE + currentSector * SECTOR_SIZE);
  sdf.seekSet(0);
  sdf.read(&track_buffer[0], SECTOR_SIZE);
}

void streamSector()
{
  uint8_t prev_timsk0_val = 0;

  // Make sure we never overflow. SteamData() stops when it fints a 0x00
  track_buffer[SECTOR_SIZE] = 0x00;

  // Only disable timer 0 interrupts
  prev_timsk0_val = TIMSK0;
  TIMSK0 = 0;

  StreamData((unsigned char*) track_buffer);
  
  // Re-enable timer 0 interrupts
  TIMSK0 = prev_timsk0_val;
}

bool openImage(char *filename )
{
  if (SD.exists(filename)) {
    //Serial.print("El archivo existe!!!\n\r\n");
  } else {
    ssd1306.cursorTo(0, 6);
    ssd1306.printString("Error: no file");
    updateStatusLine(filename);
    return false;
  }
  sdf = SD.open(filename, FILE_READ);
  if (sdf) {
    Serial.print(F("File opened fine!\r\n"));
  } else {
    ssd1306.cursorTo(0, 6);
    ssd1306.printString("Error: opening file");
    updateStatusLine(filename);
  }

  Serial.flush();
  return true;
}

void setBootImage()
{
  // Save image name to boot.txt for next boot.
  File boot = SD.open("boot.txt", FILE_WRITE);
  boot.seek(0);
  boot.write(g_current_image, MAX_FILENAME_SIZE);
  boot.close();
}

void updateStatusLine(char *msg)
{
  char status[20];

  static uint8_t currentTrack = 0;
  currentTrack = ((track_pos+3)>>2);
  memset(status, ' ', 20);
  sprintf(status, "T%02d ", currentTrack);
  for(uint8_t i = 0; i < 12; i++) {
    if (msg[i] > 0) {
      status[4+i] = msg[i];
    }
  }
  status[17] = 0;
  ssd1306.cursorTo(0, 7);
  ssd1306.printString(status);
}

void refreshDisplay()
{
  uint8_t line = 0;
  File dir;

  ssd1306.clear();

  // Dir of SD Card
  dir = SD.open("/");
  while(true) {
    File entry =  dir.openNextFile();
    char filename[MAX_FILENAME_SIZE];
    if (!entry) {
      break;
    }
    entry.getName(filename, MAX_FILENAME_SIZE - 1);
    if (strstr(filename, ".nib")) {
      if (line >= g_current_line && line <= g_current_line + 4) {
        if (line == g_current_line) {
          memset(g_current_image, 0, MAX_FILENAME_SIZE);
          strncpy(g_current_image, filename, MAX_FILENAME_SIZE-1);
        }
        
        ssd1306.cursorTo(10, line - g_current_line + 1);
        ssd1306.printString(filename);
      }
      line++;
    }
  }
  // Mark selected
  ssd1306.cursorTo(0, 1);
  ssd1306.printChar(0x10);

  // Show status line
  updateStatusLine(g_current_image);
}

void handleKeyPressed(uint8_t keycode)
{
  // read images from selected image.
  uint8_t num_images = countDiskImages();

  if (keycode == KEY_CODE_UP && g_current_line > 0) {
    g_current_line--;
    refreshDisplay();
  } else if (keycode == KEY_CODE_DOWN && g_current_line < num_images - 1) {
    g_current_line++;
    refreshDisplay();
  } else if (keycode == KEY_CODE_ENTER) {
    openImage(g_current_image);
    setBootImage();
    updateStatusLine("Image loaded.");
  }

  return;
}

int getCurrentKey()
{
  int key_value;

  key_value = analogRead(KEY_PIN);

  if (key_value > 900) {
    return 0;
  }

  if (key_value > 650) {
    return KEY_CODE_UP;
  }

  if (key_value > 400) {
    return KEY_CODE_ENTER;
  }

  return KEY_CODE_DOWN;
}

int countDiskImages()
{
  File dir;

  int count = 0;
  // Dir of SD Card
  dir = SD.open("/");
  while(true) {
    File entry =  dir.openNextFile();
    char filename[32];
    if (!entry) {
      break;
    }
    entry.getName(filename, 31);
    if (strstr(filename, ".nib")) {
      count++;
    }
  }

  return count;
}