#include <QC2Control.h>

//Pin 4 for Data+
//Pin 5 for Data-
//See How to connect in the documentation for more details.
QC2Control quickCharge(4, 5);

void setup() {
  //Optional
  //quickCharge.begin();

  //set voltage to 12V
  //quickCharge.set12V();
  //Same as
  quickCharge.setVoltage(12);

  delay(1000);
}

void loop() {
}
