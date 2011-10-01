/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 LeafLabs LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *****************************************************************************/

/**
 *  @brief Wire library, ported from Arduino. Provides a simplistic
 *  interface to i2c.
 */

#include "Wire.h"
#include "wirish.h"

/* low level conventions:
 * - SDA/SCL idle high (expected high)
 * - always start with i2c_delay rather than end
 */
uint32 i2c_delay = 1;

void i2c_start(SoftPort port) {
    I2C_DELAY;
    digitalWrite(port.sda,LOW);
    I2C_DELAY;
    digitalWrite(port.scl,LOW);
}

void i2c_stop(SoftPort port) {
    I2C_DELAY;
    digitalWrite(port.scl,HIGH);
    I2C_DELAY;
    digitalWrite(port.sda,HIGH);
}

boolean i2c_get_ack(SoftPort port) {
    I2C_DELAY;
    digitalWrite(port.scl,LOW);
    I2C_DELAY;
    digitalWrite(port.sda,HIGH);
    I2C_DELAY;
    digitalWrite(port.scl,HIGH);
    I2C_DELAY;

    if (!digitalRead(port.sda)) {
      I2C_DELAY;
      digitalWrite(port.scl,LOW);
      return true;
    } else {
      I2C_DELAY;
      digitalWrite(port.scl,LOW);
      return false;
    }
}

void i2c_send_ack(SoftPort port) {
    I2C_DELAY;
    digitalWrite(port.sda,LOW);
    I2C_DELAY;
    digitalWrite(port.scl,HIGH);
    I2C_DELAY;
    digitalWrite(port.scl,LOW);
}

void i2c_send_nack(SoftPort port) {
    I2C_DELAY;
    digitalWrite(port.sda,HIGH);
    I2C_DELAY;
    digitalWrite(port.scl,HIGH);
}

uint8 i2c_shift_in(SoftPort port) {
    uint8 data = 0;

    int i;
    for (i = 0; i < 8; i++) {
        I2C_DELAY;
        digitalWrite(port.scl,HIGH);
        I2C_DELAY;
        data += digitalRead(port.sda) << (7-i);
        I2C_DELAY;
        digitalWrite(port.scl,LOW);
    }

    return data;
}

void i2c_shift_out(SoftPort port, uint8 val) {
    int i;
    for (i = 0; i < 8; i++) {
        I2C_DELAY;
        digitalWrite(port.sda, !!(val & (1 << (7 - i))));
        I2C_DELAY;
        digitalWrite(port.scl, HIGH);
        I2C_DELAY;
        digitalWrite(port.scl, LOW);
    }
}

TwoWire::TwoWire() {
    i2c_delay = 0;
    rx_buf_idx = 0;
    rx_buf_len = 0;
    tx_buf_idx = 0;
    msg_stack_idx = 0;
    tx_buf_overflow = false;
}

/*
 * Sets pins SDA and SCL to OUPTUT_OPEN_DRAIN, joining I2C bus as
 * master.  If you want them to be some other pins, use begin(uint8,
 * uint8);
 */
void TwoWire::begin() {
    begin(0x00, PORTI2C1, SDA, SCL);
}

/*
 * Joins I2C bus as master on given SDA and SCL pins.
 */
void TwoWire::begin(uint8 master_address) {
    begin(master_address, PORTI2C1, 255, 255);
}

/*
 * Joins I2C bus as master on given SDA and SCL pins.
 */
void TwoWire::begin(uint8 master_address, i2c_type i2ctype) {
    uint8 temp[2]={255};
    if (porttype == I2CSOFT) {
        temp[0]=SDA;
        temp[1]=SCL;
    }
    begin(master_address, i2ctype, temp[0], temp[1]);
}

/*
 * Joins I2C bus as master on given SDA and SCL pins.
 */
void TwoWire::begin(uint8 master_address, i2c_type i2ctype, uint8 var1) {
    uint8 temp=255;
    if (porttype == I2CSOFT) {
        var1=SDA;
        temp=SCL;
    }
    begin(master_address, i2ctype, var1, temp);
}

/*
 * Joins I2C bus as master on given SDA and SCL pins.
 */
void TwoWire::begin(uint8 master_address, i2c_type i2ctype,
                    uint8 var1, uint8 var2) {
    porttype=i2ctype;
    if (var1 == 255) {
        var1 = 0;
    }
    if (porttype == I2CSOFT) {
        port.sda = var1;
        port.scl = var2;
        pinMode(port.scl, OUTPUT_OPEN_DRAIN);
        pinMode(port.sda, OUTPUT_OPEN_DRAIN);
        digitalWrite(port.scl, HIGH);
        digitalWrite(port.sda, HIGH);
    }
    else if (porttype == PORTI2C1) {
        i2c_master_enable(I2C1, var1);
    }
    else if (porttype == PORTI2C2) {
        i2c_master_enable(I2C2, var1);
    }
    msg_stack[0].addr = 0x00;
    msg_stack_idx = 0;
    tx_buf_idx = 0;
    tx_buf_overflow = false;
    rx_buf_idx = 0;
    rx_buf_len = 0;
}

void TwoWire::beginTransmission(uint8 slave_address) {
    if ((msg_stack[msg_stack_idx].addr != slave_address) &&
        (msg_stack[msg_stack_idx].addr != 0x00)) {
        msg_stack_idx++;
    }
    msg_stack[msg_stack_idx].addr = slave_address;
    msg_stack[msg_stack_idx].data = &tx_buf[tx_buf_idx];
    msg_stack[msg_stack_idx].length = 0;
}

void TwoWire::beginTransmission(int slave_address) {
    beginTransmission((uint8)slave_address);
}

uint8 TwoWire::endTransmission(void) {
    int8 res = 0;
    if (tx_buf_overflow) {
        return EDATA;
    }
    if (porttype == I2CSOFT){
        handleI2CMsgs(msg_stack, (msg_stack_idx+1));
    }
    else if (porttype == PORTI2C1) {
        res = i2c_master_xfer(I2C1, msg_stack, (msg_stack_idx+1), 0);
        if(res != 0) {
            i2c_disable(I2C1);
            i2c_master_enable(I2C1, I2C_BUS_RESET);
            //delayMicroseconds(5);
        }
    }
    else if (porttype == PORTI2C2) {
        res = i2c_master_xfer(I2C2, msg_stack, (msg_stack_idx+1), 0);
        if(res != 0) {
            i2c_disable(I2C2);
            i2c_master_enable(I2C2, I2C_BUS_RESET);
            //delayMicroseconds(5);
        }
    }
    tx_buf_idx = 0;
    tx_buf_overflow = false;
    msg_stack_idx = 0;
    msg_stack[0].addr = 0x00;
    return SUCCESS;
}

//TODO: Add the ability to queue messages (adding a boolean to end of function call, allows for the Arduino style to stay while also giving the flexibility to bulk send
uint8 TwoWire::requestFrom(uint8 address, int num_bytes) {
    int8 res = 0;
    if (num_bytes > WIRE_BUFSIZ) {
        num_bytes = WIRE_BUFSIZ;
    }
    if (msg_stack_idx < WIRE_STACKSIZ) {
        msg_stack_idx++;
    }
    else {
        return 0;
    }
    msg_stack[msg_stack_idx].addr = address;
    msg_stack[msg_stack_idx].flags = I2C_MSG_READ;
    msg_stack[msg_stack_idx].length = num_bytes;
    msg_stack[msg_stack_idx].data = &rx_buf[rx_buf_idx];
    if (porttype == I2CSOFT) {
        handleI2CMsgs(&msg_stack[msg_stack_idx], 1);
    }
    else if (porttype == PORTI2C1) {
        res=i2c_master_xfer(I2C1, &msg_stack[msg_stack_idx], 1, 0);
        if(res != 0) {
            i2c_disable(I2C1);
            i2c_master_enable(I2C1, I2C_BUS_RESET);
        }
    }
    else if (porttype == PORTI2C2) {
        res=i2c_master_xfer(I2C2, &msg_stack[msg_stack_idx], 1, 0);
        if(res != 0) {
            i2c_disable(I2C2);
            i2c_master_enable(I2C2, I2C_BUS_RESET);
        }
    }
    rx_buf_len = msg_stack[msg_stack_idx].xferred;
    msg_stack_idx--;
    return rx_buf_len;
}

uint8 TwoWire::requestFrom(int address, int numBytes) {
    return TwoWire::requestFrom((uint8)address, (uint8) numBytes);
}

void TwoWire::send(uint8 value) {
    if (tx_buf_idx == WIRE_BUFSIZ) {
        tx_buf_overflow = true;
        return;
    }
    tx_buf[tx_buf_idx++] = value;
    msg_stack[msg_stack_idx].length++;
}

void TwoWire::send(uint8* buf, int len) {
    for (uint8 i = 0; i < len; i++) {
        send(buf[i]);
    }
}

void TwoWire::send(int value) {
    send((uint8)value);
}

void TwoWire::send(int* buf, int len) {
    send((uint8*)buf, (uint8)len);
}

void TwoWire::send(char* buf) {
    uint8 *ptr = (uint8*)buf;
    while (*ptr) {
        send(*ptr);
        ptr++;
    }
}

uint8 TwoWire::available() {
    return rx_buf_len - rx_buf_idx;
}

uint8 TwoWire::receive() {
    if (rx_buf_idx == rx_buf_len){
        rx_buf_idx = 0;
        rx_buf_len = 0;
        return 0;
    }
    else if (rx_buf_idx == (rx_buf_len-1)){
        uint8 temp = rx_buf[rx_buf_idx++];
        rx_buf_idx = 0;
        rx_buf_len = 0;
        return temp;
    }
    return rx_buf[rx_buf_idx++];
}

// private methods

uint8 TwoWire::handleI2CMsgs(i2c_msg *msgs, int num) {
    
    uint16 msg_ptr = 0;
    while(msg_ptr < num) {
        //Recieving
        if (msgs[msg_ptr].flags == I2C_MSG_READ) {
            msgs[msg_ptr].xferred = 0;
            while (msgs[msg_ptr].xferred < msgs[msg_ptr].length) {
                if (!readOneByte(msgs[msg_ptr].addr, msgs[msg_ptr].data +
                    msgs[msg_ptr].xferred)) {
                    msgs[msg_ptr].xferred++;
                }
                else {
                    break;
                }
            }
            if (num > 1) {
                rx_buf_len+=msgs[msg_ptr].xferred;
            }
        }
        //sending
        else {
            i2c_start(port);

            i2c_shift_out(port, (msgs[msg_ptr].addr << 1) | I2C_WRITE);
            if (!i2c_get_ack(port)) {
                return ENACKADDR;
            }
            
            // shift out the address we're transmitting to
            for (uint8 i = 0; i < msgs[msg_ptr].length; i++) {
                uint8 ret = writeOneByte(msgs[msg_ptr].data[i]);
                if (ret) {
                    return ret;    // SUCCESS is 0
                }
            }
            i2c_stop(port);
        }
        msg_ptr++;
    }
    
    return SUCCESS;
}

uint8 TwoWire::writeOneByte(uint8 byte) {
    i2c_shift_out(port, byte);
    if (!i2c_get_ack(port)) {
        return ENACKADDR;
    }
    return SUCCESS;
}

uint8 TwoWire::readOneByte(uint8 address, uint8 *byte) {
    i2c_start(port);

    i2c_shift_out(port, (address << 1) | I2C_READ);
    if (!i2c_get_ack(port)) {
        return ENACKADDR;
    }

    *byte = i2c_shift_in(port);

    i2c_send_nack(port);
    i2c_stop(port);

    return SUCCESS;      // no real way of knowing, but be optimistic!
}

// Declare the instance that the users of the library can use
TwoWire Wire;

