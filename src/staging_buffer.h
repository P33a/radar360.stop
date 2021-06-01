/* Copyright (c) 2017  Paulo Costa
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

#ifndef STAGING_BUFFER_H
#define STAGING_BUFFER_H

#include "Arduino.h"

class staging_buffer_t
{
  uint8_t* buffer;
  int buf_size, free_pos;
  public:

  void (*handle_full_buffer)(uint8_t* buffer);

  staging_buffer_t();

  void init(uint8_t* new_buffer, void (*handle_full_buffer_funtion)(uint8_t* b), int new_buf_size);

  void sendFloat(char c, float v);
  void send(char c, int32_t v);
  void send(char c, byte addr, uint16_t high_word, uint16_t low_word);
  void send(char c, byte addr, byte b3, byte b2, byte b1, byte b0);

  void sendHexNibble(byte b);
  void sendHexByte(byte b);
  void sendByte(byte b);

  int get_size(void);
  int get_max_size(void);
  int get_free_space(void);
  int is_empty(void);
  uint8_t* get_buffer(void);

  void empty(void);

};

#endif // STAGING_BUFFER_H
