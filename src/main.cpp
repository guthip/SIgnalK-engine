// engine
// SensESP application to take engine data and send to SignalK server
//

// *********************************************************************************************
// Includes
// *********************************************************************************************

// Engine hat
#include "eh_analog.h"
#include "eh_digital.h"
// Sensors
#include <Adafruit_ADS1X15.h>
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
// Transforms
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/transforms/debounce.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/linear.h"
// SensESP and SignalK
#include "sensesp/signalk/signalk_output.h"
#include "sensesp_app_builder.h"
#include "sensesp_onewire/onewire_temperature.h"


#include <secrets.h>

// *********************************************************************************************
// Constants
// *********************************************************************************************


// I2C pins on SH-ESP32
const int kSDAPin = 16;
const int kSCLPin = 17;

// ADS1115 I2C address
const int kADS1115Address = 0x4b;

// CAN bus (NMEA 2000) pins on SH-ESP32
const int kCANRxPin = GPIO_NUM_34;
const int kCANTxPin = GPIO_NUM_32;

// Engine hat digital input pins
const int kDigitalInputPin1 = GPIO_NUM_15;
const int kDigitalInputPin2 = GPIO_NUM_13;
const int kDigitalInputPin3 = GPIO_NUM_14;
const int kDigitalInputPin4 = GPIO_NUM_12;

// OneWire pin
uint8_t kOneWirePin = 4;
const float kToffset = 0.0;

// Define how often SensESP should read the sensor(s) in milliseconds
  uint kReadDelay = 500;


TwoWire* i2c;

using namespace sensesp;

// *********************************************************************************************
// Convenience function to print the addresses found on the I2C bus
// *********************************************************************************************

void ScanI2C(TwoWire* i2c) {
  uint8_t error, address;

  Serial.println("Scanning...");

  for (address = 1; address < 127; address++) {
    i2c->beginTransmission(address);
    error = i2c->endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("");
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  Serial.println("done");
}

// *********************************************************************************************
// Curve Interpolators
// *********************************************************************************************

// Tank1: fresh water tank in V-berth
// V-shaped on both X and Y axis, sender is also rather a-linear
// hence a highly slanted conversion
class FuelTankConverter : public CurveInterpolator {
 public:
  FuelTankConverter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table tp translate the ohm values returned by
    // our fluid level sender to percentage tank fill
    clear_samples();
    // addSample(CurveInterpolator::Sample(inValue, outValue));
    add_sample(CurveInterpolator::Sample(0.00, 0.00)); // assure proper bottom
    add_sample(CurveInterpolator::Sample(1.02, 0.25));
    add_sample(CurveInterpolator::Sample(2.00, 0.50));
    add_sample(CurveInterpolator::Sample(3.00, 0.75));
    add_sample(CurveInterpolator::Sample(4.00, 1.00)); // assure proper top
  }
};


reactesp::ReactESP app;


  // *********************************************************************************************
  // The setup function performs one-time application initialization.
  // *********************************************************************************************
void setup() {
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif


  // initialize the I2C bus
  i2c = new TwoWire(0);
  i2c->begin(kSDAPin, kSCLPin);

  ScanI2C(i2c);

// OneWire interface
  DallasTemperatureSensors* dts = new DallasTemperatureSensors(kOneWirePin);

  // Initialize ADS1115 - ADC on Engine Hat
  auto ads1115 = new Adafruit_ADS1115();
  ads1115->setGain(GAIN_ONE);
  bool ads_initialized = ads1115->begin(kADS1115Address, i2c);
  debugD("ADS1115 initialized: %d", ads_initialized);


  // *********************************************************************************************
  // Setup WiFi network connection
  // *********************************************************************************************

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("engine")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi(WIFI_SSD, WIFI_PW)
                    //->set_sk_server(SERVER_IP, SERVER_PORT)
                    ->enable_ota(OTA_PW)
                    ->get_app();


  // Connect the tank senders
  auto fueltank_level = ConnectTankSender(ads1115, 0, "0");
  // auto tank_b_volume = ConnectTankSender(ads1115, 1, "B");
  // auto tank_c_volume = ConnectTankSender(ads1115, 2, "C");
  // auto tank_d_volume = ConnectTankSender(ads1115, 3, "D");

  // Connect the tacho senders
  auto tacho_1_frequency = ConnectTachoSender(kDigitalInputPin1, "mainEngine");

  // Connect the alarm inputs
  // auto alarm_2_input = ConnectAlarmSender(kDigitalInputPin2, "2");
  // auto alarm_3_input = ConnectAlarmSender(kDigitalInputPin3, "3");
  // auto alarm_4_input = ConnectAlarmSender(kDigitalInputPin4, "4");

  // Update the alarm states based on the input value changes
  //alarm_2_input->connect_to(
  //    new LambdaConsumer<bool>([](bool value) { alarm_states[1] = value; }));
  // alarm_3_input->connect_to(
  //     new LambdaConsumer<bool>([](bool value) { alarm_states[2] = value; }));
  // alarm_4_input->connect_to(
  //     new LambdaConsumer<bool>([](bool value) { alarm_states[3] = value; }));

  // Connect OneWIre temperature sensors
  auto* tempTamb =
      new OneWireTemperature(dts, kReadDelay, "/ambientTemperature/oneWire"); // Ambient engine room
  auto* tempTexhaust =
      new OneWireTemperature(dts, kReadDelay, "/exhaustTemperature/oneWire"); // Exhaust elbow
  auto* tempTcoolant =
      new OneWireTemperature(dts, kReadDelay, "/coolantTemperature/oneWire"); // Heat exchanger


  // *********************************************************************************************
  // Fuel Tank
  // *********************************************************************************************

  // Connect the analog input to Signal K output. This will publish the
  // analog input value to the Signal K server every time it changes.

  // Create a new Analog Input Sensor that reads an analog input pin periodically.

  fueltank_level
    ->connect_to(new FuelTankConverter("/tanks.0.converter"))
    ->connect_to(new SKOutputFloat(
      "tanks.fuel.0.currentLevel",                // Signal K path
      "tanks/fuel/0/currentLevel",                // configuration path, used in the
                                                  // web UI and for storing the
                                                  // configuration
      new SKMetadata("",                          // Define output units
                     "Fuel tank current level")   // Value description
      ));


 // *********************************************************************************************
// Tacho sensor = Engine revolutions
 // *********************************************************************************************
  tacho_1_frequency
//    ->connect_to(new LambdaConsumer<float>(
//        [](float value) { PrintValue(display, 3, "RPM 1", 60 * value); }
    ->connect_to(new SKOutputFloat(
      "propulsion.mainEngine.revolutions",   // Signal K path
      "propulsion/mainEngine/revolutions",   // configuration path
      new SKMetadata("",                            // Define output units
                     "Engine revolutions")  // Value description
      ));


  // *********************************************************************************************
  // Temperature sensors
   // To find valid Signal K Paths that fits your need you look at this link:
  // https://signalk.org/specification/1.4.0/doc/vesselsBranch.html
  // *********************************************************************************************

  // Engine temperature: engine room ambient
  tempTamb
    ->connect_to(new Linear(1.0, 0.0, "/ambientTemperature/linear"))
    ->connect_to(new SKOutputFloat(
      "propulsion.mainEngine.ambientTemperature",   // Signal K path
      "propulsion/mainEngine/ambientTemperature",   // configuration path
      new SKMetadata("K",                            // Define output units
                     "Engine ambient temperature")  // Value description
      ));

  // Engine temperature: engine exhaust
  tempTexhaust
    ->connect_to(new Linear(1.0, 0.0, "/exhaustTemperature/linear"))
    ->connect_to(new SKOutputFloat(
      "propulsion.mainEngine.exhaustTemperature",   // Signal K path
      "propulsion/mainEngine/exhaustTemperature",   // configuration path
      new SKMetadata("K",                            // Define output units
                     "Engine exhaust temperature")  // Value description
      ));

  // Engine temperature: engine coolant
  tempTcoolant
    ->connect_to(new Linear(1.0, 0.0, "/coolantTemperature/linear"))
    ->connect_to(new SKOutputFloat(
      "propulsion.mainEngine.coolantTemperature",   // Signal K path
      "propulsion/mainEngine/coolantTemperature",   // configuration path
      new SKMetadata("K",                            // Define output units
                     "Engine coolant temperature")  // Value description
      ));


  // *********************************************************************************************
  // Add an observer that prints out the current value of the  inputs
  // every time it changes.
  // *********************************************************************************************
  tempTamb->attach([tempTamb]() {
    debugD("T ambient: %f", tempTamb->get() - 272.15 + kToffset);
    });
  tempTexhaust->attach([tempTexhaust]() {
    debugD("T exhaust: %f", tempTexhaust->get() - 272.15 + kToffset);
    });
  tempTcoolant->attach([tempTcoolant]() {
    debugD("T coolant: %f", tempTcoolant->get() - 272.15 + kToffset);
    });

  tacho_1_frequency->attach([tacho_1_frequency]() {
    debugD("***Tacho 1: %f", tacho_1_frequency->get());
    });

  fueltank_level->attach([fueltank_level]() {
    debugD("***Fuel: %f", fueltank_level->get());
    });


  // *********************************************************************************************
  // Start networking, SK server connections and other SensESP internals
  // *********************************************************************************************
  sensesp_app->start();
}

// *********************************************************************************************
// loop()
// *********************************************************************************************
void loop() { app.tick(); }
