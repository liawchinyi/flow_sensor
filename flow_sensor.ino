#include <arduinoFFT.h>
#include <Arduino.h>

#define SAMPLES 1024
#define ADC_PIN A3
#define LED1_PIN 24
#define LED2_PIN 25

#define AVERAGE_COUNT 4  // Number of FFTs to average for noise reduction

volatile bool newDataAvailable = false;

const double samplingFrequency = 5000;                                // Hz
const unsigned long samplingPeriod_us = 1000000 / samplingFrequency;  // Sampling period in microseconds

double vReal[SAMPLES];
double vImag[SAMPLES];
volatile int sampleIndex = 0;

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, samplingFrequency);

struct repeating_timer adcTimer;
struct repeating_timer ledTimer;

// Timer interrupt function for ADC sampling
bool onADCTimer(struct repeating_timer *t) {
  if (sampleIndex < SAMPLES) {
    vReal[sampleIndex] = analogRead(ADC_PIN);
    vImag[sampleIndex] = 0;  // Ensure imaginary part is zero
    sampleIndex++;
  } else {
    newDataAvailable = true;
    return false;  // Stop the timer
  }
  return true;  // Continue the timer
}

// Timer interrupt function for toggling LED1
bool onLEDTimer(struct repeating_timer *t) {
  static bool led1State = LOW;
  led1State = !led1State;
  digitalWrite(LED1_PIN, led1State);
  return true;  // Continue the timer
}

void setup() {
  Serial.begin(115200);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);

  // Set up the repeating timer to toggle LED1 every 1 second
  add_repeating_timer_ms(1000, onLEDTimer, NULL, &ledTimer);  // 1000ms (1 second) interval

  // Set up the ADC sampling timer
  add_repeating_timer_us(samplingPeriod_us, onADCTimer, NULL, &adcTimer);
}

void loop() {
  static double avgMagnitude[SAMPLES / 2] = {0};
  static int avgCount = 0;

  if (newDataAvailable) {
    newDataAvailable = false;
    sampleIndex = 0;

    // Remove DC offset
    double meanValue = 0;
    for (int i = 0; i < SAMPLES; i++) {
      meanValue += vReal[i];
    }
    meanValue /= SAMPLES;
    for (int i = 0; i < SAMPLES; i++) {
      vReal[i] -= meanValue;
    }

    FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.complexToMagnitude(vReal, vImag, SAMPLES);

    // Sum the magnitudes for averaging
    for (int i = 0; i < SAMPLES / 2; i++) {
      avgMagnitude[i] += vReal[i];
    }
    avgCount++;

    if (avgCount >= AVERAGE_COUNT) {
      double maxMagnitude = 0;
      double maxFrequency = 0;

      // Calculate the average magnitudes and find the max
      for (int i = 0; i < SAMPLES / 2; i++) {
        avgMagnitude[i] /= AVERAGE_COUNT;
        double frequency = (i * samplingFrequency) / SAMPLES;

        if (avgMagnitude[i] > maxMagnitude) {
          maxMagnitude = avgMagnitude[i];
          maxFrequency = frequency;
        }
      }

      // Toggle LED2 every FFT calculation
      static bool led2State = LOW;
      led2State = !led2State;
      digitalWrite(LED2_PIN, led2State);

      // Map the frequency to flow rate (this is a placeholder, use your sensor's calibration data)
      double flowRate = maxFrequency * 10; // Example: Flow rate in L/min = frequency * factor

      // Print the maximum magnitude, its corresponding frequency, and the estimated flow rate
      Serial.print("Max Magnitude: ");
      Serial.print(maxMagnitude);
      Serial.print(", Frequency: ");
      Serial.print(maxFrequency);
      Serial.print(", Estimated Flow Rate: ");
      Serial.print(flowRate);
      Serial.println(" L/min");

      // Reset averaging counters
      memset(avgMagnitude, 0, sizeof(avgMagnitude));
      avgCount = 0;
    }

    // Restart the ADC sampling timer
    add_repeating_timer_us(samplingPeriod_us, onADCTimer, NULL, &adcTimer);
  }

  delay(10);  // Optional delay to avoid flooding the serial output
}
