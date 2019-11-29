#include <Arduino.h>
#include "configuration.h"
#include "../data/heart.h"

//#include "SerialCommands.h"

#include "colormaps.h"

#include <ESP8266WiFi.h>
//#include <aREST.h>

//#include "wifisecret.h"

#include <RunningAverage.h>


const uint8_t PanelWidth = 16;  // 16 pixel x 16 pixel matrix of leds
const uint8_t PanelHeight = 16;
const uint8_t TileWidth = 2;  // laid out in 2 panels x 2 panels mosaic
const uint8_t TileHeight = 1;
MyMosaic mosaic(PanelWidth, PanelHeight, TileWidth, TileHeight);

// make sure to set this to the correct pins
const uint8_t DotDataPin = 2;
const uint16_t PixelCount = PanelWidth * PanelHeight * TileWidth * TileHeight;
MyBus strip(PixelCount, DotDataPin);


char serial_command_buffer_[32];
// SerialCommands serial_commands_(&Serial, serial_command_buffer_, sizeof(serial_command_buffer_), "\r\n", " ");
// bool runAnimation = true;

// aREST rest = aREST();
// #define LISTEN_PORT 80


// WiFiServer server(LISTEN_PORT);

int toggleLeds(String command)
{
	strip.ClearTo(RgbColor(255,255,255));
	return 1;
}

RgbColor RgbColorF(float r, float g, float b)
{
	return RgbColor(r*colorSaturation, g*colorSaturation, b*colorSaturation);
}

RunningAverage myRA(10);
float averageCycleDelay = 0;

void setup()
{
    Serial.begin(115200);
    while (!Serial); // wait for serial attach
	
	// rest.function("led", toggleLeds);
	// rest.variable("Average cycle delay", &averageCycleDelay);
	// rest.variable("Animations running", &runAnimation);
	
	// rest.set_id("1");
	// rest.set_name("lightserver");
	
	// WiFi.begin(WIFI_SSID,WIFI_PASS);
	// while (WiFi.status() != WL_CONNECTED)
	// {
	// 	delay(500);
	// 	Serial.print(".");
	// }
	Serial.println("");
	Serial.println("WiFi connected");
	
	// server.begin();
	Serial.println("Server started");
	
	Serial.println(WiFi.localIP());
	
	// serial_commands_.SetDefaultHandler(&cmd_unrecognized);
    // for (int i=0;i<nCommands;++i)
	// {
	// 	serial_commands_.AddCommand(_commands[i]);
	// }

    // this resets all the neopixels to an off state
    strip.Begin();
    strip.ClearTo(RgbColorF(0,0,0));
    strip.Show();
}

float currentTime = 0;
float delta;
void updateTimers()
{
	float time = (float)millis() / 1000.0f;
    delta = time - currentTime;
    currentTime = time;
}


int updateVarDelay = 50;
int varCount = 0;
NeoGamma<NeoGammaEquationMethod> colorGamma;


const uint16_t left = 0;
const uint16_t right = PanelWidth * TileWidth - 1;
const uint16_t top = 0;
const uint16_t bottom = PanelHeight * TileHeight - 1;

RgbColor red(128, 0, 0);
RgbColor green(0, 128, 0);
RgbColor blue(0, 0, 128);
RgbColor white(128);
RgbColor black(0);
RgbColor yellow(65,0, 60);

int heartLite = 'A';
int heartLiteMin = 'C';
int heartLiteMax = 'A'+9;
bool hearLiteInc = true;
unsigned long lastTick = 0;
int cPanel = 0;

void loop()
{
 	// strip.SetPixelColor(mosaic.Map(left, top), white);
 	// strip.SetPixelColor(mosaic.Map(right, top), red);
 	// strip.SetPixelColor(mosaic.Map(right, bottom), green);
 	// strip.SetPixelColor(mosaic.Map(left, bottom), blue);
	unsigned int read = 0;
	short x = 0;
	short y=0;
	const float min = 'A';
	const float max = 'U';
	
	while (read < heart_txt_len)
	{
		// Skip windows linefeed
		if (heart_txt[read] == 0x0d)
		{
			x=0;
			++y;
			++read;
		}
		if (heart_txt[read] == 0x0a)
		{
			++read;
			continue;
		}

		
		unsigned char cVal = heart_txt[read];
		float color = (max-cVal+min);
		const float maxColor = 100;
		
		
		short idx = mosaic.Map(x+PanelWidth*cPanel,y);

		if (cVal > heartLite)
		{
			RgbColor clr2(yellow);
			if (rand() % 2 )
				clr2.Darken(rand()%30);
			else
				clr2.Lighten(rand()%20);
			strip.SetPixelColor(idx, colorGamma.Correct(clr2));
		}
		else
		{
			strip.SetPixelColor(idx, colorGamma.Correct(red));
		}
		
		++read;
		++x;
	}

	// Check for heartbeat
	unsigned long delay = 100 + 35 * (heartLite-heartLiteMin);
	unsigned long tick = millis();
	if (tick > (lastTick+delay) )
	{
		lastTick = tick;
		if (hearLiteInc)
			++heartLite;
		else
			--heartLite;

		if (heartLite == heartLiteMax)
		{
			heartLite = heartLiteMax;
			hearLiteInc = false;
		}
		if (heartLite == heartLiteMin)
		{
			heartLite = heartLiteMin;
			hearLiteInc = true;
		}
	}

	cPanel =  (cPanel==0) ? 1:0;

	// WiFiClient client = server.available();
	// if (client )
	// {
	// 	while (!client.available())
	// 	{
	// 		delay(1);
	// 	}
	// 	rest.handle(client);
	// }

	updateTimers();
	myRA.addValue(delta);
	if (varCount == updateVarDelay)
	{
		varCount=0;
		averageCycleDelay = myRA.getFastAverage();
	}
	else
	{
		++varCount;
	}
	
	// serial_commands_.ReadSerial();
    strip.Show();
}
