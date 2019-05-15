//Programmed by Quan Qing, Dec. 2018
//All rights reserved.

//Qing LAB at ASU: http://qinglab.physics.asu.edu
//quan.qing@asu.edu

#include <Wire.h>

const float RELAYCONTROL_VER = 0.2;
const char * cmd_list[] = {"HELLO", "RESET", "SET_ON", "SET_OFF", "SET_RELAYS_STATUS", "READ_RELAYS_STATUS", "SCANI2C"}; //, "SET_DIGITAL_HIGH", "SET_DIGITAL_LOW", "v", "q"};
// HELLO for the first time is handshake, afterwards nothing
// RESET sets all relay to off, and expect to wait for HELLO handshake message
// SET_ON/SET_OFF + channel sets single relay channel on/off
// SET_RELAYS_STATUS + unsigned long number, sets all relays' status with each bit representing one of the 32 channels
// READ_RELAYS_STATUS, reads status of all channels, returns a number that has bits showing relay's on/off state

const char termchar = '\n';
const int maxbuflen = 128; //maximum length of command string
char read_buf[maxbuflen];
volatile int cur_pos = 0;

const int FLAG_WAIT_FOR_STR = 0;
const int FLAG_STR_DETECTED = 1;
const int FLAG_PROCESS_FINISHED = 2;
volatile int buf_flag = FLAG_WAIT_FOR_STR;

const unsigned char I2C_SegAddr = 0x20;
const unsigned char I2C_A1_SETBIT = 0;
const unsigned char I2C_A2_SETBIT = 0;
const unsigned char I2C_bank1_addr = I2C_SegAddr | (I2C_A1_SETBIT << 1) | (I2C_A2_SETBIT << 2);
const unsigned char I2C_bank2_addr = (I2C_SegAddr | (I2C_A1_SETBIT << 1) | (I2C_A2_SETBIT << 2)) + 1;
const byte REG_IODIRA = 0x00;
const byte REG_IODIRB = 0x01;
const byte REG_GPIOA = 0x12;
const byte REG_GPIOB = 0x13;

volatile unsigned relay_bank1_state = 0; //16 bits
volatile unsigned relay_bank2_state = 0; //16 bits

const int MIN_DIG_PIN = 2;
const int MAX_DIG_PIN = 7;

void setup() {
  // put your setup code here, to run once:
  int i;

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  for (i = MIN_DIG_PIN; i <= MAX_DIG_PIN; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  Serial.begin(9600);
  Wire.begin();
  reset_relay();

  //Serial.print("RELAY CONTROLLER INITIALIZED. VERSION ");
  //Serial.println(RELAYCONTROL_VER);
  Serial.flush();
}

void loop() {
  // put your main code here, to run repeatedly:
  static int state = 0;
  //serialEvent() will take charge in filling the buffers and keep track of buf_flag to proper state.

  parse_cmd(&state);
}

void I2C_write_reg(unsigned char addr, byte reg, byte value)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  int error = Wire.endTransmission();
  if (error != 0) {
    Serial.print("I2C write error at address 0x");
    Serial.print(addr, HEX);
    Serial.print(" with register 0x");
    Serial.print(reg, HEX);
    Serial.print(" for value 0x");
    Serial.print(value, HEX);
    Serial.print(". Error code: 0x");
    Serial.println(error, HEX);
  }
}

void Scan_I2C()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");
  nDevices = 0;
  for (address = 0; address <= 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission(true);

    if (address < 16) Serial.print("0");
    Serial.print(address, HEX);
    if (error == 0) {
      Serial.print("[found!]");
      nDevices++;
    } else {
      Serial.print("["); Serial.print(error); Serial.print("]     ");
    }
    Serial.print( (address % 8) == 7 ? "\n" : " ");   
  }
  if (nDevices == 0){
    Serial.println("No I2C devices were found");
  }
  else{
    Serial.print(nDevices);
    Serial.println(" I2C devices were found.");
  }
}

int next_token(char * cmd, int startpos, int * pstr_start, int * pstr_end)
{
  int i;

  i = startpos;
  if (pstr_start == NULL || pstr_end == NULL) {
    return -1;
  }
  *pstr_start = -1;
  *pstr_end = -1;

  while (!isGraph(cmd[i]) && cmd[i] != 0 && i < maxbuflen) {
    i++;
  }
  *pstr_start = i;
  while (isGraph(cmd[i]) && cmd[i] != 0 && i < maxbuflen) {
    i++;
  }
  *pstr_end = i;

  if (*pstr_start >= 0 && *pstr_end > *pstr_start) {
    return 0;
  }
  else {
    return -1;
  }
}

int cmd_index(char * str, int len)
{
  int i;

  for (i = 0; i < sizeof(cmd_list) / sizeof(char*); i++) {
    if (strncmp(cmd_list[i], str, len) == 0) {
      return i;
    }
  }
  return -1;
}

int parse_cmd(int * pstate) //state 0 means waiting for start signal
{
  int p = 0, q = 0;
  int idx = -1;
  unsigned long channel;
  unsigned long digital_pin;

  if (buf_flag == FLAG_STR_DETECTED) {
    if (next_token(read_buf, q, &p, &q) >= 0) { //first token is used for command
      idx = cmd_index(read_buf + p, q - p);
      switch (idx) {
        case 0: //handshake hello
          if (*pstate == 0) {
            Serial.println("READY!");
            *pstate = 1;
            digitalWrite(LED_BUILTIN, HIGH);
          }
          else {
            Serial.println("OK");
          }
          break;
        case 1: //reset relay
          reset_relay();
          Serial.println("RESET OK");
          *pstate = 0;
          digitalWrite(LED_BUILTIN, LOW);
          break;
        case 2: //SET_ON
          channel = strtoul(read_buf + q, NULL, 0);
          if (set_relay(channel, 1) == 0) {
            Serial.print("SET_ON [");
            Serial.print(channel);
            Serial.println("] OK");
          }
          else {
            Serial.println("?");
          }
          break;
        case 3: //SET_OFF
          channel = strtoul(read_buf + q, NULL, 0);
          if (set_relay(channel, 0) == 0) {
            Serial.print("SET_OFF [");
            Serial.print(channel);
            Serial.println("] OK");
          }
          else {
            Serial.println("?");
          }
          break;
        case 4: //SET_RELAYS_STATUS
          channel = strtoul(read_buf + q, NULL, 0);
          channel &= 0xFFFFFFFFUL;

          relay_bank1_state = (unsigned)(channel & 0xFFFFUL);
          relay_bank2_state = (unsigned)((channel >> 16) & 0xFFFFUL);

          update_relay_state(I2C_bank1_addr, relay_bank1_state);
          update_relay_state(I2C_bank2_addr, relay_bank2_state);
          Serial.println("OK");
          break;
        case 5: //READ_RELAY_STATUS
          Serial.println((unsigned long)(((unsigned long)relay_bank2_state << 16) | (unsigned long)relay_bank1_state));
          break;
        case 6: //SCANI2C
          Scan_I2C();
          break;
        default:
          //Serial.println("?");
          break;
      }
    }
    buf_flag = FLAG_PROCESS_FINISHED;
  }
  Serial.flush();
  return idx;
}

void reset_relay()
{
  I2C_write_reg(I2C_bank1_addr, REG_IODIRA, 0x00);  //bank 1, IODIRA, all pins configured as output
  I2C_write_reg(I2C_bank1_addr, REG_IODIRB, 0x00);  //bank 1, IODIRB, all pins configured as output
  //Serial.println("Setting bank1 output ok");
  I2C_write_reg(I2C_bank2_addr, REG_IODIRA, 0x00);  //bank 2, IODIRA, all pins configured as output
  I2C_write_reg(I2C_bank2_addr, REG_IODIRB, 0x00);  //bank 2, IODIRB, all pins configured as output
  //Serial.println("Setting bank2 output ok");
  relay_bank1_state = 0;
  relay_bank2_state = 0;

  update_relay_state(I2C_bank1_addr, relay_bank1_state);
  //Serial.println(I2C_bank1_addr);
  //Serial.println("resetting bank1...");
  update_relay_state(I2C_bank2_addr, relay_bank2_state);
  //Serial.println(I2C_bank2_addr);
  //Serial.println("resetting bank2...");
}

void update_relay_state(unsigned char addr, unsigned state)
{
  //Serial.println("updating relay with the following address and ioports:");
  //Serial.println(addr);
  //Serial.println(state);
  //Serial.println("ports and data:");
  //Serial.println(REG_GPIOA);
  //Serial.println((byte)(state & 0xFF));
  I2C_write_reg(addr, REG_GPIOA, (byte)(state & 0xFF));
  //Serial.println(REG_GPIOB);
  //Serial.println((byte)((state & 0xFF00) >> 8));
  I2C_write_reg(addr, REG_GPIOB, (byte)((state & 0xFF00) >> 8));
}

int set_relay(unsigned long channel, int b)
{
  unsigned data = 1;

  if (channel >= 0 && channel <= 15) {
    data <<= channel;

    if (b == 1) { //turning relay on
      relay_bank1_state |= data;
    }
    else { //turning relay off
      relay_bank1_state &= ~data;
    }

    update_relay_state(I2C_bank1_addr, relay_bank1_state);
    return 0;
  }
  else if (channel >= 16 && channel <= 31) {
    data <<= channel - 16;

    if (b == 1) { //turning relay on
      relay_bank2_state |= data;
    }
    else { //turning relay off
      relay_bank2_state &= ~data;
    }

    update_relay_state(I2C_bank2_addr, relay_bank2_state);
    return 0;
  }
  else {
    return -1;
  }
}

void serialEvent()
{
  const int CONTINUE_BIT = 0x01;
  const int DEQUEUE_BIT = 0x02;

  int flag = CONTINUE_BIT;
  byte c;

  while (((flag & CONTINUE_BIT) != 0) && (Serial.available() > 0)) {

    c = Serial.peek();

    switch (buf_flag) {
      case FLAG_WAIT_FOR_STR: //reading is expected
        if (c != termchar) {
          if (cur_pos < maxbuflen - 1) {
            read_buf[cur_pos] = (char)c;
            cur_pos++;
            flag = CONTINUE_BIT | DEQUEUE_BIT; //continue loop, char should be removed from queue
          }
          else {
            read_buf[maxbuflen - 1] = 0; //close the string
            cur_pos = maxbuflen - 1;
            buf_flag = FLAG_STR_DETECTED;
            flag = 0; //buffer is full, not continue reading, not taking the char off
          }
        }
        else {
          if (cur_pos >= maxbuflen - 1) {
            cur_pos = maxbuflen - 1;
          }
          read_buf[cur_pos] = 0; //close the string
          flag = DEQUEUE_BIT; //termchar detected, not continue reading, take the char off
          buf_flag = FLAG_STR_DETECTED;
        }
        break;
      case FLAG_STR_DETECTED:
        break;
      case FLAG_PROCESS_FINISHED: //process is done and restoring reading status
        cur_pos = 0;
        read_buf[0] = 0;
        buf_flag = FLAG_WAIT_FOR_STR;
        flag = CONTINUE_BIT; //not remove char, leave to the next loop
        break;
      default:
        break;
    }//switch

    if ((flag & DEQUEUE_BIT) != 0) { //remove char from queue
      c = Serial.read();
    }

  }//while
}
