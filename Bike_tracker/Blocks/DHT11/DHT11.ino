#include <dht.h>
dht DHT;

void setup(){
  Serial.begin(9600);
}
void loop()
{
  int chk = DHT.read11(9);
  Serial.print("Temperature = ");
  Serial.print(DHT.temperature);
  Serial.println("C");
  Serial.print("Humidity = ");
  Serial.print(DHT.humidity);
  Serial.println("%");
  delay(2000);
}

