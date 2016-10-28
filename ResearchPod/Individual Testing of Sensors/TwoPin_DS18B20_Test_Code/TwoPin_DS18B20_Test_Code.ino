//
// FILE: TwoPin_DS18B20.ino
// AUTHOR: Rob Tillaart
// VERSION: 0.1.00
// PURPOSE: two pins for two sensors demo
// DATE: 2014-06-13
// URL: http://forum.arduino.cc/index.php?topic=216835.msg1764333#msg1764333
//
// Released to the public domain
//

#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS_1 4

OneWire oneWire_in(ONE_WIRE_BUS_1);

DallasTemperature sensor_inhouse(&oneWire_in);

void setup(void)
{
    Serial.begin(9600);
    Serial.println("Dallas Temperature Control Library Demo - TwoPin_DS18B20");

    sensor_inhouse.begin();
}

void loop(void)
{
    Serial.print("Requesting temperatures...");
    sensor_inhouse.requestTemperatures();
    Serial.println(" done");

    Serial.print("Inhouse: ");
    Serial.println(sensor_inhouse.getTempCByIndex(0));

}
