#include <Time.h>

extern unsigned long timer0_millis;
Time timer1;

void setup()
{
	pinMode(5, OUTPUT);
	digitalWrite(5, HIGH);
	delay(500);
	//initMillis();
	timer0_millis = 4294962296UL;
	timer1.setInterval(200);
}

void loop() {
	if (timer1.checkTime() == 1) {
		digitalWrite(5, !digitalRead(5));
	}
}