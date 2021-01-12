#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <compat/twi.h>
#include "Arduino.h" // for digitalWrite and micros
#include "pins_arduino.h"
#include "twi.h"

static volatile uint8_t twi_state;
static volatile uint8_t twi_slarw;
static volatile bool twi_sendStop;      // should the transaction end with a stop
static volatile bool twi_inRepStart;     // in the middle of a repeated start

// static uint32_t twi_timeout_us = 8000000; // ~ aboute 1 sec, max - 0x00FFFFFF ~ 16 sec
// if twi_timeout_us = 0 then endless loop

static volatile bool twi_timeout_off_flag = true;      // turn off timeout
static volatile bool twi_timed_out_flag = false;       // a timeout has been seen
static volatile bool twi_do_reset_on_timeout = false;  // reset the TWI registers on timeout

static void (*twi_onSlaveTransmit)(void);
static void (*twi_onSlaveReceive)(uint8_t*, int);

static uint8_t twi_masterBuffer[TWI_BUFFER_LENGTH];
static volatile uint8_t twi_masterBufferIndex;
static volatile uint8_t twi_masterBufferLength;

static uint8_t twi_txBuffer[TWI_BUFFER_LENGTH];
static volatile uint8_t twi_txBufferIndex;
static volatile uint8_t twi_txBufferLength;

static uint8_t twi_rxBuffer[TWI_BUFFER_LENGTH];
static volatile uint8_t twi_rxBufferIndex;

static volatile uint8_t twi_error;

static volatile uint8_t set_1 = 0;
static volatile uint8_t set_2 = 0x12;
static volatile uint8_t set_3 = 0x8A;

//*************************************************************************************

void twi_init(void){
  // initialize state
  twi_state = TWI_READY;
  twi_sendStop = true;    // default value
  twi_inRepStart = false;
  volatile uint8_t *out;
  // activate internal pullups for twi.
  ATOMIC_BLOCK (ATOMIC_FORCEON){
    out = portOutputRegister (digitalPinToPort (SDA)); 
    *out |= (digitalPinToBitMask (SDA) | digitalPinToBitMask (SCL));
  }
  // initialize twi prescaler and bit rate
  TWSR = 1;  // TWI Bit Rate Prescaler = 4
  /* 
     TWI Bit Rate for 16MHz MCU
     400 kHz - TWBR = 3
     100 kHz - TWBR = 18
      10 kHz - TWBR = 198
  */
  TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;
  // enable twi module, acknowledge bit, and twi interrupt
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
}

void twi_disable(void){
  volatile uint8_t *out;
  // disable twi module
  TWCR = 0;
  // deactivate internal pullups for twi.
  ATOMIC_BLOCK (ATOMIC_FORCEON){
    out = portOutputRegister (digitalPinToPort (SDA)); 
    *out &= (~ digitalPinToBitMask (SDA) & ~ digitalPinToBitMask (SCL));
  }
}

void twi_setAddress(uint8_t address){
  // set twi slave address (skip over TWGCE bit)
  TWAR = address << 1;
}

void twi_setFrequency(uint32_t frequency){
  //!!! избавиться от 32 битного аргумета функции, возможно сделать switch()
  if (frequency > 400000) frequency = 400000;
  if (frequency >= 10000) {
    TWSR = 1;  // TWI Bit Rate Prescaler = 4
    TWBR = F_CPU / frequency - 16 >> 3;
  }
  if (frequency < 500) frequency = 500;
  if (frequency < 10000){
    TWSR = 3;  // TWI Bit Rate Prescaler = 64
    TWBR = F_CPU / frequency - 16 >> 7;
  }
  /* twi bit rate formula from atmega128 manual pg 204
  SCL Frequency = CPU Clock Frequency / (16 + (2 * TWBR * Prescaler))*/
  
}

uint8_t twi_transmit(const uint8_t* data, uint8_t length){
  uint8_t *p = &twi_txBuffer;
  
  // !!! нужно избавиться от 3х буферов
  
  // ensure data will fit into buffer
  if(TWI_BUFFER_LENGTH < (twi_txBufferLength+length)) return 1;
  
  // ensure we are currently a slave transmitter
  if(TWI_STX != twi_state) return 2;
  
  // set length and copy data into tx buffer
  memcpy(p + twi_txBufferLength, data, length);
  twi_txBufferLength += length;
  
  return 0;
}

void twi_attachSlaveRxEvent( void (*function)(uint8_t*, int) ) {twi_onSlaveReceive = function;}

void twi_attachSlaveTxEvent( void (*function)(void) ) {twi_onSlaveTransmit = function;}

inline void twi_reply(bool ack){
  // transmit master read ready signal, with or without ack
  if(ack){
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
  }else{
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT);
  }
}

void twi_stop(void){
  volatile uint8_t counter_1;
  volatile uint8_t counter_2;
  volatile uint8_t counter_3;
  // send stop condition
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWSTO);
  // wait for stop condition to be exectued on bus
  // TWINT is not set after a stop condition!
  if (!twi_timeout_off_flag){
    counter_1 = set_1;
    counter_2 = set_2;
    counter_3 = set_3;
  }  
check:                                        // написать это без goto пока не хватает ума. :)
  if (!(TWCR & _BV(TWSTO))) goto go;
  if (twi_timeout_off_flag) goto check;
  if (!(--counter_1)) goto check;
  if (!(--counter_2)) goto check;
  if (!(--counter_3)) goto check;
  twi_handleTimeout(twi_do_reset_on_timeout); // с этим подумать, возможно выставить флаг здесь.
  return; //timeout
go:
  // update twi state
  twi_state = TWI_READY;
}

void twi_releaseBus(void){
  // release bus
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT);
  // update twi state
  twi_state = TWI_READY;
}

void twi_setTimeoutInMicros(uint32_t timeout, bool reset_with_timeout){
  twi_timed_out_flag = false;
  // twi_timeout_us = timeout;
  twi_do_reset_on_timeout = reset_with_timeout;
  if (timeout == 0) {                        // variants (timeout < 100)
    twi_timeout_off_flag = true;
  }else{
    twi_timeout_off_flag = false;
    set_1 = timeout & 0x0000FFUL;
    set_2 = (timeout & 0x00FF00UL) >> 8;
    set_3 = (timeout & 0xFF0000UL) >> 16;
  }
}

void twi_handleTimeout(bool reset){ 
  twi_timed_out_flag = true; 

  if (reset) {
    // remember bitrate and address settings
    uint8_t previous_TWBR = TWBR;
    uint8_t previous_TWAR = TWAR;

    // reset the interface
    twi_disable();
    twi_init();

    // reapply the previous register values
    TWAR = previous_TWAR;
    TWBR = previous_TWBR;
  }
}

bool twi_manageTimeoutFlag(bool clear_flag){ 
  bool flag = twi_timed_out_flag; // why it before clear flag ???
  if (clear_flag){
    twi_timed_out_flag = false;
  }
  return flag;
}

uint8_t twi_writeTo(uint8_t address, uint8_t* data, uint8_t length, bool wait, bool sendStop){
  volatile uint8_t counter_1;
  volatile uint8_t counter_2;
  volatile uint8_t counter_3;
  // ensure data will it into buffer
  if(TWI_BUFFER_LENGTH < length) return 1;

  // wait until twi is ready, become master transmitter
  if (!twi_timeout_off_flag){
    counter_1 = set_1;
    counter_2 = set_2;
    counter_3 = set_3;
  }
check1:                                        // написать это без goto пока не хватает ума. :)
  if (TWI_READY == twi_state) goto go1;
  if (twi_timeout_off_flag) goto check1;
  if (!(--counter_1)) goto check1;
  if (!(--counter_2)) goto check1;
  if (!(--counter_3)) goto check1;
  twi_handleTimeout(twi_do_reset_on_timeout); // с этим подумать, возможно выставить флаг здесь.
  return 5; //timeout
go1:  
  twi_state = TWI_MTX;
  twi_sendStop = sendStop;
  // reset error state (0xFF.. no error occured)
  twi_error = 0xFF;

  // initialize buffer iteration vars
  twi_masterBufferIndex = 0;
  twi_masterBufferLength = length;
  // copy data to twi buffer
  memcpy(&twi_masterBuffer, data, length); 
   
  // build sla+w, slave device address + w bit (TW_WRITE = 0)
  twi_slarw = address << 1;
  
  // if we're in a repeated start, then we've already sent the START
  // in the ISR. Don't do it again.
  //
  if (twi_inRepStart) {
    // if we're in the repeated start state, then we've already sent the start,
    // (@@@ we hope), and the TWI statemachine is just waiting for the address byte.
    // We need to remove ourselves from the repeated start state before we enable interrupts,
    // since the ISR is ASYNC, and we could get confused if we hit the ISR before cleaning
    // up. Also, don't enable the START interrupt. There may be one pending from the 
    // repeated start that we sent outselves, and that would really confuse things.
    twi_inRepStart = false;      // remember, we're dealing with an ASYNC ISR
    TWDR = twi_slarw;
    
    if (!twi_timeout_off_flag){
      counter_1 = set_1;
      counter_2 = set_2;
      counter_3 = set_3;
    }
check2:                                        // написать это без goto пока не хватает ума. :)
    if (!(TWCR & _BV(TWWC))) goto go2;
    if (twi_timeout_off_flag) goto check2;
    if (!(--counter_1)) goto check2;
    if (!(--counter_2)) goto check2;
    if (!(--counter_3)) goto check2;
    twi_handleTimeout(twi_do_reset_on_timeout); // с этим подумать, возможно выставить флаг здесь.
    return 5; //timeout
go2:
    
    TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE);  // enable INTs, but not START
  } else {
    // send start condition
    TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE) | _BV(TWSTA); // enable INTs
  }

  // wait for write operation to complete
  if (!twi_timeout_off_flag){
    counter_1 = set_1;
    counter_2 = set_2;
    counter_3 = set_3;
  }
check3:                                        // написать это без goto пока не хватает ума. :)
  if (!(wait && (TWI_MTX == twi_state))) goto go3;
  if (twi_timeout_off_flag) goto check3;
  if (!(--counter_1)) goto check3;
  if (!(--counter_2)) goto check3;
  if (!(--counter_3)) goto check3;
  twi_handleTimeout(twi_do_reset_on_timeout);  // с этим подумать, возможно выставить флаг здесь.
  return 5; //timeout
go3:
  if (twi_error == 0xFF)
    return 0; // success
  else if (twi_error == TW_MT_SLA_NACK)
    return 2; // error: address send, nack received
  else if (twi_error == TW_MT_DATA_NACK)
    return 3; // error: data send, nack received
  else
    return 4; // other twi error
}

uint8_t twi_readFrom(uint8_t address, uint8_t* data, uint8_t length, bool sendStop){
  volatile uint8_t counter_1;
  volatile uint8_t counter_2;
  volatile uint8_t counter_3;
  // ensure data will fit into buffer
  if(TWI_BUFFER_LENGTH < length)return 0;
  // wait until twi is ready, become master receiver
  if (!twi_timeout_off_flag){
    counter_1 = set_1;
    counter_2 = set_2;
    counter_3 = set_3;
  }
check1:                                        // написать это без goto пока не хватает ума. :)
  if (TWI_READY == twi_state) goto go1;
  if (twi_timeout_off_flag) goto check1;
  if (!(--counter_1)) goto check1;
  if (!(--counter_2)) goto check1;
  if (!(--counter_3)) goto check1;
  twi_handleTimeout(twi_do_reset_on_timeout); // с этим подумать, возможно выставить флаг здесь.
  return 0; //timeout
go1:
  twi_state = TWI_MRX;
  twi_sendStop = sendStop;
  // reset error state (0xFF.. no error occured)
  twi_error = 0xFF;
  // initialize buffer iteration vars
  twi_masterBufferIndex = 0;
  twi_masterBufferLength = length-1;  // This is not intuitive, read on...
  // On receive, the previously configured ACK/NACK setting is transmitted in
  // response to the received byte before the interrupt is signalled. 
  // Therefor we must actually set NACK when the _next_ to last byte is
  // received, causing that NACK to be sent in response to receiving the last
  // expected byte of data.

  // build sla+w, slave device address + w bit
  twi_slarw = (address << 1) + 1;
  
  if (twi_inRepStart) {
    // if we're in the repeated start state, then we've already sent the start,
    // (@@@ we hope), and the TWI statemachine is just waiting for the address byte.
    // We need to remove ourselves from the repeated start state before we enable interrupts,
    // since the ISR is ASYNC, and we could get confused if we hit the ISR before cleaning
    // up. Also, don't enable the START interrupt. There may be one pending from the 
    // repeated start that we sent ourselves, and that would really confuse things.
    twi_inRepStart = false;      // remember, we're dealing with an ASYNC ISR
    if (!twi_timeout_off_flag){
      counter_1 = set_1;
      counter_2 = set_2;
      counter_3 = set_3;
    }
check2:                                        // написать это без goto пока не хватает ума. :)
    if (!(TWCR & _BV(TWWC))) goto go2;
    if (twi_timeout_off_flag) goto check2;
    if (!(--counter_1)) goto check2;
    if (!(--counter_2)) goto check2;
    if (!(--counter_3)) goto check2;
    twi_handleTimeout(twi_do_reset_on_timeout); // с этим подумать, возможно выставить флаг здесь.
    return 0; //timeout
go2:
    TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE);  // enable INTs, but not START
  } else {
    // send start condition
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);
  }
  // wait for read operation to complete
  if (!twi_timeout_off_flag){
    counter_1 = set_1;
    counter_2 = set_2;
    counter_3 = set_3;
  }
check3:                                        // написать это без goto пока не хватает ума. :)
  if (!(TWI_MRX == twi_state)) goto go3;
  if (twi_timeout_off_flag) goto check3;
  if (!(--counter_1)) goto check3;
  if (!(--counter_2)) goto check3;
  if (!(--counter_3)) goto check3;
  twi_handleTimeout(twi_do_reset_on_timeout);  // с этим подумать, возможно выставить флаг здесь.
  return 0; //timeout
go3:
  if (twi_masterBufferIndex < length) {
    length = twi_masterBufferIndex;
  }
  // copy twi buffer to data
  memcpy(data, &twi_masterBuffer, length);
  return length;
}

ISR(TWI_vect){
  switch(TW_STATUS){
    // All Master
    case TW_START:     // sent start condition
    case TW_REP_START: // sent repeated start condition
      // copy device address and r/w bit to output register and ack
      TWDR = twi_slarw;
      twi_reply(1);
      break;

    // Master Transmitter
    case TW_MT_SLA_ACK:  // slave receiver acked address
    case TW_MT_DATA_ACK: // slave receiver acked data
      // if there is data to send, send it, otherwise stop 
      if(twi_masterBufferIndex < twi_masterBufferLength){
        // copy data to output register and ack
        TWDR = twi_masterBuffer[twi_masterBufferIndex++];
        twi_reply(1);
      }else{
        if (twi_sendStop){
          twi_stop();
       } else {
         twi_inRepStart = true;  // we're gonna send the START
         // don't enable the interrupt. We'll generate the start, but we
         // avoid handling the interrupt until we're in the next transaction,
         // at the point where we would normally issue the start.
         TWCR = _BV(TWINT) | _BV(TWSTA)| _BV(TWEN) ;
         twi_state = TWI_READY;
        }
      }
      break;
    case TW_MT_SLA_NACK:  // address sent, nack received
      twi_error = TW_MT_SLA_NACK;
      twi_stop();
      break;
    case TW_MT_DATA_NACK: // data sent, nack received
      twi_error = TW_MT_DATA_NACK;
      twi_stop();
      break;
    case TW_MT_ARB_LOST: // lost bus arbitration
      twi_error = TW_MT_ARB_LOST;
      twi_releaseBus();
      break;

    // Master Receiver
    case TW_MR_DATA_ACK: // data received, ack sent
      // put byte into buffer
      twi_masterBuffer[twi_masterBufferIndex++] = TWDR;
      __attribute__ ((fallthrough));
    case TW_MR_SLA_ACK:  // address sent, ack received
      // ack if more bytes are expected, otherwise nack
      if(twi_masterBufferIndex < twi_masterBufferLength){
        twi_reply(1);
      }else{
        twi_reply(0);
      }
      break;
    case TW_MR_DATA_NACK: // data received, nack sent
      // put final byte into buffer
      twi_masterBuffer[twi_masterBufferIndex++] = TWDR;
      if (twi_sendStop){
        twi_stop();
      } else {
        twi_inRepStart = true;  // we're gonna send the START
        // don't enable the interrupt. We'll generate the start, but we
        // avoid handling the interrupt until we're in the next transaction,
        // at the point where we would normally issue the start.
        TWCR = _BV(TWINT) | _BV(TWSTA)| _BV(TWEN) ;
        twi_state = TWI_READY;
      }
      break;
    case TW_MR_SLA_NACK: // address sent, nack received
      twi_stop();
      break;
    // TW_MR_ARB_LOST handled by TW_MT_ARB_LOST case

    // Slave Receiver
    case TW_SR_SLA_ACK:   // addressed, returned ack
    case TW_SR_GCALL_ACK: // addressed generally, returned ack
    case TW_SR_ARB_LOST_SLA_ACK:   // lost arbitration, returned ack
    case TW_SR_ARB_LOST_GCALL_ACK: // lost arbitration, returned ack
      // enter slave receiver mode
      twi_state = TWI_SRX;
      // indicate that rx buffer can be overwritten and ack
      twi_rxBufferIndex = 0;
      twi_reply(1);
      break;
    case TW_SR_DATA_ACK:       // data received, returned ack
    case TW_SR_GCALL_DATA_ACK: // data received generally, returned ack
      // if there is still room in the rx buffer
      if(twi_rxBufferIndex < TWI_BUFFER_LENGTH){
        // put byte in buffer and ack
        twi_rxBuffer[twi_rxBufferIndex++] = TWDR;
        twi_reply(1);
      }else{
        // otherwise nack
        twi_reply(0);
      }
      break;
    case TW_SR_STOP: // stop or repeated start condition received
      // ack future responses and leave slave receiver state
      twi_releaseBus();
      // put a null char after data if there's room
      if(twi_rxBufferIndex < TWI_BUFFER_LENGTH){
        twi_rxBuffer[twi_rxBufferIndex] = '\0';
      }
      // callback to user defined callback
      twi_onSlaveReceive(twi_rxBuffer, twi_rxBufferIndex);
      // since we submit rx buffer to "wire" library, we can reset it
      twi_rxBufferIndex = 0;
      break;
    case TW_SR_DATA_NACK:       // data received, returned nack
    case TW_SR_GCALL_DATA_NACK: // data received generally, returned nack
      // nack back at master
      twi_reply(0);
      break;
    
    // Slave Transmitter
    case TW_ST_SLA_ACK:          // addressed, returned ack
    case TW_ST_ARB_LOST_SLA_ACK: // arbitration lost, returned ack
      // enter slave transmitter mode
      twi_state = TWI_STX;
      // ready the tx buffer index for iteration
      twi_txBufferIndex = 0;
      // set tx buffer length to be zero, to verify if user changes it
      twi_txBufferLength = 0;
      // request for txBuffer to be filled and length to be set
      // note: user must call twi_transmit(bytes, length) to do this
      twi_onSlaveTransmit();
      // if they didn't change buffer & length, initialize it
      if(0 == twi_txBufferLength){
        twi_txBufferLength = 1;
        twi_txBuffer[0] = 0x00;
      }
      __attribute__ ((fallthrough));      
      // transmit first byte from buffer, fall
    case TW_ST_DATA_ACK: // byte sent, ack returned
      // copy data to output register
      TWDR = twi_txBuffer[twi_txBufferIndex++];
      // if there is more to send, ack, otherwise nack
      if(twi_txBufferIndex < twi_txBufferLength){
        twi_reply(1);
      }else{
        twi_reply(0);
      }
      break;
    case TW_ST_DATA_NACK: // received nack, we are done 
    case TW_ST_LAST_DATA: // received ack, but we are done already!
      // ack future responses
      twi_reply(1);
      // leave slave receiver state
      twi_state = TWI_READY;
      break;

    // All
    case TW_NO_INFO:   // no state information
      break;
    case TW_BUS_ERROR: // bus error, illegal stop/start
      twi_error = TW_BUS_ERROR;
      twi_stop();
      break;
  }
}
