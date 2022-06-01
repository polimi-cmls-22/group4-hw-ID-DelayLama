/*
 ____  _____ _        _    
| __ )| ____| |      / \   
|  _ \|  _| | |     / _ \  
| |_) | |___| |___ / ___ \ 
|____/|_____|_____/_/   \_\

The platform for ultra-low latency audio and sensor processing

http://bela.io

A project of the Augmented Instruments Laboratory within the
Centre for Digital Music at Queen Mary University of London.
http://www.eecs.qmul.ac.uk/~andrewm

(c) 2016 Augmented Instruments Laboratory: Andrew McPherson,
	Astrid Bin, Liam Donovan, Christian Heinrichs, Robert Jack,
	Giulio Moro, Laurel Pardue, Victor Zappi. All rights reserved.

The Bela software is distributed under the GNU Lesser General Public License
(LGPL 3.0), available here: https://www.gnu.org/licenses/lgpl-3.0.txt
*/

/*
 *	USING A CUSTOM RENDER.CPP FILE FOR PUREDATA PATCHES - LIBPD
 *  ===========================================================
 *  ||                                                       ||
 *  || OPEN THE ENCLOSED _main.pd PATCH FOR MORE INFORMATION ||
 *  || ----------------------------------------------------- ||
 *  ===========================================================
 */

#include <Bela.h>
#include <cmath>
#include <I2c_Codec.h>
#include <array>
#include <stdio.h>
#include "Adafruit_TCS34725.h"
#include <libraries/Scope/Scope.h>
#include <libraries/Oscillator/Oscillator.h>
#include <libraries/ADSR/ADSR.h>
#include <libraries/PulseIn/PulseIn.h>
#include <libraries/OscReceiver/OscReceiver.h>
#include <libraries/OscSender/OscSender.h>
#include <libraries/Biquad/Biquad.h>

OscReceiver oscReceiver;
OscSender oscSender;
int localPort = 7562;
int remotePort = 7563;
const char* remoteIp = "192.168.6.1";

PulseIn distanceSensor;
ADSR envelope; // ADSR envelope
float gAttack = 0.5; // Envelope attack (seconds)
float gDecay = 2; // Envelope decay (seconds)
float gRelease = 2; // Envelope release (seconds)
float gSustain = 0.75; // Envelope sustain level
bool gate=true;

const int Trigger = 10;
const int Echo = 11;



std::array<Oscillator, 4> oscillators;

Adafruit_TCS34725 tcs34725;   // Object to handle TCS34725 sensing
AuxiliaryTask i2cTask;		// Auxiliary task to read I2C

// Change this to change how often the TCS34725 is read (in Hz)
int readInterval = 50;
int readCount = 0;			// How long until we read again...
int readIntervalSamples = 0; // How many samples between reads
float r, g, b, c;

enum COLOR {
	RED,
	GREEN,
	BLUE,
	TEAL,
	PURPLE,
	YELLOW
};

COLOR lastColor = RED;
std::string chord = "Major";

float RED_MIN = 255;
float GREEN_MIN = 255;
float BLUE_MIN = 255;
float RED_MAX = 0;
float GREEN_MAX = 0;
float BLUE_MAX = 0;

//This is the min delta a channel must have over the others
//in order to "recognize" one color as such
//Feel free to experiment with this
//It is given as a fraction of the full re-mapped range
const uint16_t CHANNEL_DELTA = 255/4;


//Declare all color thresholds here
const float RED_THRESHOLD[3] = {255, 0, 0};
const float BLUE_THRESHOLD[3] = {0, 55, 255};
const float GREEN_THRESHOLD[3] = {60, 255, 60};
const float YELLOW_THRESHOLD[3] = {150, 150, 40};
const float PURPLE_THRESHOLD[3] = {120, 50, 150};
const float TEAL_THRESHOLD[3] = {50, 200, 130};

std::array<float, 4> freqs = {440.0f, 440.0f*2, 440.0f*3, 440.0f*4};
std::array<float, 4> amps = {0.25f, 0.25f, 0.25f, 0.25f};
std::array<float, 4> outs = {0.0f, 0.0f, 0.0f, 0.0f};
float gAmplitude = 0.3;
int gCount = 0;
float gInterval = 0.06;
float gOut = 0.0;
int gPulseTriggerCount = 0;
int gPulseTriggerIntervalMS = 60;
int gPulseTriggerIntervalSamples;
int gPulseMinLength = 7;
float gPulseRescale = 58;
float distance = 0;
float minDistance = 2;
float maxDistance = 100;
float maxFreq = 2000;
float minFreq = 50;


//Moving average filter state for distance
float distOld1 = 0, distOld2 = 0;

float filterOut = 0;

Biquad bqFilter;


void readTCS34725(void*);
void setChordFromRGB();
void updatePitch();
void setThresholds();
void updateBounds();
void remapRGBForCalibration();
void setRed();
void setBlue();
bool compareColor(const float threshold[3]);
void setGreen();
void setTeal();
void setPurple();
void setYellow();
void calculate_coefficients(float sampleRate, float frequency, float q);


bool setup(BelaContext *context, void *userData)
{
	
	BiquadCoeff::Settings s = {
		.fs = context->audioSampleRate,
		.q = 0.8,
		.peakGainDb = 0,
		.type = BiquadCoeff::bandpass,
		.cutoff = 500
	};
	
	bqFilter.setup(s);
	
	//begin TCS34725
    if(!tcs34725.begin()) {
        rt_printf("Error initialising tcs34725\n");
        return false;
    }
    
    
    
    //set RGB sensor parameters
    tcs34725.setIntegrationTime(TCS34725_INTEGRATIONTIME_154MS);
	tcs34725.setGain(TCS34725_GAIN_16X);

   
    //start auxiliary read task

	Bela_scheduleAuxiliaryTask(Bela_createAuxiliaryTask(&readTCS34725, 50, "bela-tcs34725"));
    
    
	// Set ADSR parameters
	envelope.setAttackRate(gAttack * context->audioSampleRate);
	envelope.setDecayRate(gDecay * context->audioSampleRate);
	envelope.setReleaseRate(gRelease * context->audioSampleRate);
	envelope.setSustainLevel(gSustain);
	
	// Set oscillators
    for(int i = 0; i < oscillators.size(); i++)
    	oscillators[i].setup(context->audioSampleRate,Oscillator::triangle);
	
	gPulseTriggerIntervalSamples = context->digitalSampleRate*gPulseTriggerIntervalMS/1000.0f;
	// Check that audio and digital have the same number of frames
	// per block, an assumption made in render()
	if(context->audioFrames != context->digitalFrames) {
		rt_fprintf(stderr, "This example needs audio and digital running at the same rate.\n");
		return false;
	}
	
	pinMode(context,0,Trigger,OUTPUT);
	pinMode(context,0,Echo,INPUT);

	distanceSensor.setup(context, Echo, HIGH);
	
	//OSC setup
	oscSender.setup(remotePort, remoteIp);
	
	
	return true;
}

// render() is called regularly at the highest priority by the audio engine.
// Input and output are given from the audio hardware and the other
// ADCs and DACs (if available). If only audio is available, numMatrixFrames
// will be 0.

void render(BelaContext *context, void *userData)
{

	for(int i = 0; i < freqs.size(); i++)
	{

		oscillators[i].setFrequency(freqs[i]);
	}
			
	envelope.gate(gate);
   	// This for() loop goes through all the samples in the block
	for (unsigned int n = 0; n < context->audioFrames; n++) {
	    
		bool state;
		gPulseTriggerCount++;
		
		if(gPulseTriggerCount==gPulseTriggerIntervalSamples){
			gPulseTriggerCount=0;
			state = HIGH;
		}else{
			state = LOW;
		}
		
		digitalWrite(context,n,Trigger,state);
		
		int pulseLength = distanceSensor.hasPulsed(context,n); //length of the pulse in samples if detected 
		float duration = 1e6 * pulseLength/context->digitalSampleRate; // convert to duration in microseconds
		if (pulseLength >= gPulseMinLength){
			distance = duration / gPulseRescale; //compute distance (in cm) according to the manual
			if(distance<maxDistance)
				//MA filtering of the distance to remove noise
				distance = (distance + distOld1 + distOld2) / 3;
				distOld2 = distOld1; //Update filter status
				distOld1 = distance;
				updatePitch();
				oscSender.newMessage("/distance");
				oscSender.add(distance);
				oscSender.send();
		}
		
		float amp = gAmplitude * envelope.process();
		gOut = 0;
		//I first store all the values then I write them
		//Don't know if it's any different if we directly write them inside the first loop
		for(int i = 0; i < freqs.size(); i++)
		{
			outs[i] = oscillators[i].process() * amps[i];
			gOut += outs[i];
		}
		
		gOut = gOut * amp;

		gOut = bqFilter.process(gOut);
		
		//write the same current sample on all output channels
		for (unsigned int channel = 0; channel < context->audioOutChannels; channel++)
				audioWrite(context,n,channel,gOut);

    }
    


}

// cleanup() is called once at the end, after the audio has stopped.
// Release any resources that were allocated in setup().

void cleanup(BelaContext *context, void *userData)
{
	
}

// Auxiliary task to read the I2C board
void readTCS34725(void*)
{
	while(!gShouldStop){
		
		
		tcs34725.getRGB(&r, &g, &b);

		//Update max and min values
		//(Calibration should improve as the execution goes on)
		updateBounds();

		//remap the rgb values according to the current bounds
		remapRGBForCalibration();

		oscSender.newMessage("/colour");
		oscSender.add(r);
		oscSender.add(g);
		oscSender.add(b);
		oscSender.send();

		setChordFromRGB();

		//Update center frequency
		if (r+g+b>0)
			bqFilter.setFc((80*b+200*g+800*r)/(r+g+b));
		
    	usleep(10000); // interval often should it run, in microseconds
    }
}

// Auxiliary task to read the I2C board
void updatePitch()
{
	//Update fundamental from distance sensor
	freqs[0] = map(distance, minDistance, maxDistance, minFreq, maxFreq);
	setChordFromRGB(); //update other freqs
	
}


void updateBounds()
{
	if (r > RED_MAX)
	{
		RED_MAX = r;

	}

	if (r < RED_MIN && r>0)
	{
		RED_MIN = r;
	}

	if (g > GREEN_MAX)
	{
		GREEN_MAX = g;
	}

	if (g < GREEN_MIN && g > 0)
	{
		GREEN_MIN = g;
	}

		if (b > BLUE_MAX)
	{
		BLUE_MAX = b;

	}

	if (b < BLUE_MIN && b > 0)
	{
		BLUE_MIN = b;
	}
}

void remapRGBForCalibration()
{
	//Remap the rgb values to full range 0-255
	//From the actual range which is being used by the sensor
	//in the current environment
	

	if (RED_MAX > RED_MIN && GREEN_MAX > GREEN_MIN && BLUE_MAX > BLUE_MIN)
	{
			r = (r - RED_MIN) * 255 / (RED_MAX - RED_MIN);
	g = (g - GREEN_MIN) * 255 / (GREEN_MAX - GREEN_MIN);
	b = (b - BLUE_MIN) * 255 / (BLUE_MAX - BLUE_MIN);
	}
	
	

}

void setChordFromRGB()
{
	


	if (compareColor(RED_THRESHOLD))
	{
		//color is assumed to be red
		setRed();

	} else if (compareColor(BLUE_THRESHOLD))
	{
		//color is assumed to be blue
		setBlue();
	} else if (compareColor(GREEN_THRESHOLD))
	{
		//color is assumed to be blue
		setGreen();

	} else if (compareColor(YELLOW_THRESHOLD))
	{
		setYellow();
	} else if (compareColor(PURPLE_THRESHOLD))
	{
		setPurple();
	} else if (compareColor(TEAL_THRESHOLD))
	{
		setTeal();
	}
	else {
		switch (lastColor)
		{
			case RED: setRed();
			break;
			case GREEN: setGreen();
			break;
			case BLUE: setBlue();
			break;
			case TEAL: setTeal();
			break;
			case PURPLE: setPurple();
			break;
			case YELLOW: setYellow();
			break;
			default: setRed();
		}
	}

	oscSender.newMessage("/chord");
	oscSender.add(chord);
	oscSender.send();
}

bool compareColor(const float threshold[3])
{
	return std::abs(threshold[0]-r) <= CHANNEL_DELTA && 
	std::abs(threshold[1]-g) <= CHANNEL_DELTA &&
	std::abs(threshold[2]-b) <= CHANNEL_DELTA;
}

void setRed()
{
		//Major chord
		freqs[1] = freqs[0] * pow(2.0f, 4.0f/12);
        freqs[2] = freqs[0] * pow(2.0f, 7.0f/12);
        freqs[3] = freqs[0] * 2;
		lastColor = RED;
		chord = "Major chord";
}

void setPurple()
{
        //Example: Dominant 7th chord
        freqs[1] = freqs[0] * pow(2.0f, 4.0f/12);
        freqs[2] = freqs[0] * pow(2.0f, 7.0f/12);
        freqs[3] = freqs[0] * pow(2.0f, 10.0f/12);
        lastColor = PURPLE;
        chord = "Dominant 7th chord";
	
}

void setBlue()
{
		//Minor 7th chord
        freqs[1] = freqs[0] * pow(2.0f, 3.0f/12);
        freqs[2] = freqs[0] * pow(2.0f, 7.0f/12);
        freqs[3] = freqs[0] * pow(2.0f, 10.0f/12);
		lastColor = BLUE;
		chord = "Minor 7th chord";
}

void setGreen()
{
		//Half diminished chord
        freqs[1] = freqs[0] * pow(2.0f, 3.0f/12);
        freqs[2] = freqs[0] * pow(2.0f, 6.0f/12);
        freqs[3] = freqs[0] * 2;
		lastColor = GREEN;
		chord = "Half diminished chord";
}


void setTeal()
{
	//7b9 chord
    freqs[1] = freqs[0] * pow(2.0f, 4.0f/12);
    freqs[2] = freqs[0] * pow(2.0f, 13.0f/12);
    freqs[3] = freqs[0] * pow(2.0f, 10.0f/12);
	lastColor = TEAL;
	chord = "Dominant 7b9 chord";
}



void setYellow()
{
    //Minor chord
    freqs[1] = freqs[0] * pow(2.0f, 3.0f/12);
    freqs[2] = freqs[0] * pow(2.0f, 7.0f/12);
    freqs[3] = freqs[0] * 2;
    lastColor = YELLOW;
    chord = "Minor chord";
}


 
