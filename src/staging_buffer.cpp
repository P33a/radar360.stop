/* Copyright (c) 2016  Paulo Costa
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */

#include "staging_buffer.h"
#include "Arduino.h"
/*
static byte isHexNibble(char c)
{
  if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F')) return 1;
  else return 0;
}

static byte HexNibbleToByte(char c)
{
  if (c >= '0' && c <= '9') return c - '0';
  else if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  else return 0;
}
*/
void staging_buffer_t::sendByte(byte b)
{
  if (free_pos >= buf_size) { // Buffer is full must be handled
    if (handle_full_buffer && buf_size) { // if we know how to do it and it is initialized
      (*handle_full_buffer)(buffer);
	  free_pos = 0;
	  }
  }
  // Now we can begin filling it again
  buffer[free_pos] = b;
  free_pos++;
}


void staging_buffer_t::sendHexNibble(byte b)
{
  if (b < 10) {
    sendByte('0' + b);
  } else if (b < 16) {
    sendByte('A' + (b - 10));
  }
}

void staging_buffer_t::sendHexByte(byte b)
{
  sendHexNibble(b >> 4);
  sendHexNibble(b & 0x0F);
}

void staging_buffer_t::send(char c, int32_t v)
{
  sendByte(c);
  sendHexByte(v >> 24);
  sendHexByte((v >> 16) & 0xFF);
  sendHexByte((v >> 8)  & 0xFF);
  sendHexByte(v & 0xFF);
}

void staging_buffer_t::send(char c, byte addr, byte b3, byte b2, byte b1, byte b0)
{
  sendByte(c);
  sendHexByte(b3);
  sendHexByte(b2);
  sendHexByte(b2);
  sendHexByte(b1);
  sendHexByte(addr);
}


void staging_buffer_t::send(char c, byte addr, uint16_t high_word, uint16_t low_word)
{
  sendByte(c);
  sendHexByte((high_word >> 8)  & 0xFF);
  sendHexByte(high_word & 0xFF);
  sendHexByte((low_word >> 8)  & 0xFF);
  sendHexByte(low_word & 0xFF);
  sendHexByte(addr);
}


void staging_buffer_t::sendFloat(char c, float v)
{
  send(c, *((int32_t*) &v));
}

staging_buffer_t::staging_buffer_t()
{
  handle_full_buffer = NULL;
  buffer = NULL;
  buf_size = 0;
  free_pos = 0;
}

void staging_buffer_t::init(uint8_t* new_buffer, void (*handle_full_buffer_funtion)(uint8_t* b), int new_buf_size)
{
  buffer = new_buffer;
  handle_full_buffer = handle_full_buffer_funtion;
  buf_size = new_buf_size;
  free_pos = 0;
}

int staging_buffer_t::get_size(void)
{
  return free_pos;
}

int staging_buffer_t::get_max_size(void)
{
  return buf_size;
}


int staging_buffer_t::get_free_space(void)
{
  return buf_size - free_pos;
}

int staging_buffer_t::is_empty(void)
{
  return free_pos == 0;
}

uint8_t* staging_buffer_t::get_buffer(void)
{
  return buffer;
}


void staging_buffer_t::empty(void)
{
  free_pos = 0;
}
