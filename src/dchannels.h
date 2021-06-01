/* Copyright (c) 2019  Paulo Costa
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

#ifndef DCHANNELS_H
#define DCHANNELS_H

#include "Arduino.h"

union dchannels_u{
  uint32_t u;
  float f;
};

class dchannels_t
{
    int8_t frameState;
    char curChannel, curSubChannel;
    char frameHexData[8];
  public:

    void (*process_frame)(char channel, char sub_channel, uint32_t value, dchannels_t& obj);
    void (*serial_write)(uint8_t b);

    dchannels_t();

    void init(void (*process_frame_function)(char channel, char sub_channel, uint32_t value, dchannels_t& obj),
              void (*serial_write_function)(uint8_t b)
              );
    void StateMachine(byte b);
    void sendFloat(char c, char sc, float v);
    void send(char c, char sc, uint32_t v);
    void send(char c, char sc, uint16_t high_word, uint16_t low_word);
    void send(char c, char sc, byte b3, byte b2, byte b1, byte b0);
    
    void sendHexNibble(byte b);
    void sendHexByte(byte b);
    void sendByte(byte b);
};

#endif // DCHANNELS_H
