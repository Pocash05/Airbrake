/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2021 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#include "vector_nav.h"

bfs::Vn300 vn;

int main() {
  Serial.begin(115200);
  while(!Serial) {}
  /* Initialize communication */
  SPI.begin();
  vn.Config(&SPI, 34);
  if (!vn.Begin()) {
    Serial.print("Error Code: ");
    Serial.println(vn.error_code());
    while (1) {}
  }
  while (1) {
    /* Read sensor and print values */
    if (vn.Read()) {
      Serial.print(vn.yaw_rad() * 180 / 3.14159);
      Serial.print("\t");
      Serial.print(vn.pitch_rad() * 180 / 3.14159);
      Serial.print("\t");
      Serial.println(vn.roll_rad() * 180 / 3.14159);
    }
    delay(50);
  }
}

