//
//  SerialPort.h
//  SerialPortSample
//
//  Created by Ciel Marks on 22/04/2015.
//
//

#ifndef DmmDriver_SerialPort_h
#define DmmDriver_SerialPort_h

#include <MacTypes.h>

long SerialAvailable();
int SerialRead();
ssize_t SerialWrite(char b);

int openSerial(char *);
void closeSerial();

void delay(int millis);

#endif // DmmDriver_SerialPort_h
