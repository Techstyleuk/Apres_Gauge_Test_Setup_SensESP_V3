/*
 * Apres Gauge Test Setup using SensESP V3
 * Copyright (c) 2026 Jason Greenwood 
 * https://www.youtube.com/@ApresSail
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
// Apres Gauge Test Setup using SensESP V3.
// Software version 3.0.1 - 20260201
// This software is designed to be used in combination with a video created for Youtube: *****
// Included in this version:
// 1.  INA219 voltage sensor which will be connected to either side of a gauge (temp, fuel, etc.), 
// this will give the values needed to calculate the resistance of the sender as long as you have the 
// resistance of the gauge
// 2.  1-Wire Sensor as a reference temperature sensor for the demonstration  
//
#include <memory>
#include "sensesp.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"
#include <Wire.h>  
#include <Adafruit_INA219.h>
#include "sensesp_onewire/onewire_temperature.h"
#include <Arduino.h>
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/analogvoltage.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/voltagedivider.h"
#include "sensesp/transforms/frequency.h"

using namespace sensesp;
using namespace sensesp::onewire;

class TempInterpreter : public CurveInterpolator {
 public:
  TempInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the gauge voltage values returned by
    // our INA219_C to Fuel Level % (Ratio)
    clear_samples();
    // addSample(CurveInterpolator::Sample(knownOhm, knownFuelLevel)); - these numbers need changing
    add_sample(CurveInterpolator::Sample(1000, 0));
    add_sample(CurveInterpolator::Sample(240, 0));
    add_sample(CurveInterpolator::Sample(218, .1));
    add_sample(CurveInterpolator::Sample(198, .2));
    add_sample(CurveInterpolator::Sample(177, .3));
    add_sample(CurveInterpolator::Sample(156, .4));
    add_sample(CurveInterpolator::Sample(135, .5));
    add_sample(CurveInterpolator::Sample(114, .6));
    add_sample(CurveInterpolator::Sample(93, .7));
    add_sample(CurveInterpolator::Sample(72, .8)); 
    add_sample(CurveInterpolator::Sample(51, .9));
    add_sample(CurveInterpolator::Sample(30, 1)); 
    add_sample(CurveInterpolator::Sample(0, 1)); 
  }
};
////////////////////INA219
  Adafruit_INA219 ina219_tg;
 
  const float Rgauge = 25; //this is the measured resistance of the gauge

  float read_TG_shuntvoltage_callback() { return (ina219_tg.getShuntVoltage_mV() / 1000);} //drop across the shunt - this is for debug 
  float read_TG_busvoltage_callback() { return (ina219_tg.getBusVoltage_V());} //downstream voltage to be supplied to the consumer - this is for debug
  float read_TG_loadvoltage_callback() { return (ina219_tg.getBusVoltage_V() + (ina219_tg.getShuntVoltage_mV() / 1000));} //loadvoltage is the battery or supply voltage
  float read_TG_r2resistance_callback() { return (ina219_tg.getBusVoltage_V() + (ina219_tg.getShuntVoltage_mV() / 1000));} //Calculation not complete yet, make this calculate R2 manually
  //////////////////////INA219 end

// The setup function performs one-time application initialization.
void setup() {
  SetupLogging(ESP_LOG_DEBUG);

  Wire.begin(21,22);                // join i2c bus (address optional for master)

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("Apres-Gauge-test-setup-V3.0.1")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi_client("My WiFi SSID", "my_wifi_password")
                    //->set_wifi_access_point("My AP SSID", "my_ap_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->get_app();

 // 3) Direct copy from old Main.ccp - Start 
/// 1-Wire Temp Sensors (amended for SensESP version 3)

  DallasTemperatureSensors* dts = new DallasTemperatureSensors(25); //digital 2

 // A Battery Temperature - electrical.batteries.A.temperature
  auto* ref_temp =
      new OneWireTemperature(dts, 1000, "/Reference Temperature/oneWire");

  ConfigItem(ref_temp)
      ->set_title("Reference Temperature")
      ->set_description("Temperature check")
      ->set_sort_order(100);
    
  auto ref_temp_calibration =
      new Linear(1.0, 0.0, "/Reference Temperature/linear");
      
  ConfigItem(ref_temp_calibration)
      ->set_title("Reference Temperature Calibration")
      ->set_description("Calibration for the Reference temperature sensor")
      ->set_sort_order(200);

  auto ref_temp_sk_output = new SKOutputFloat(
      "environment.reference.temperature", "/Reference Temperature/skPath");

  ConfigItem(ref_temp_sk_output)
      ->set_title("Reference Temperature Signal K Path")
      ->set_description("Signal K path for the Reference temperature")
      ->set_sort_order(300);

  ref_temp->connect_to(ref_temp_calibration)
      ->connect_to(ref_temp_sk_output);

///////////////////////////INA219 start
  ina219_tg.begin();  // Initialize second board with the address 0x45 ----> A1 and A0 soldered = 0x45

//ina219_D
  //// Fuel Sender Config ////

  auto* ina219_TG_shuntvoltage = 
      new RepeatSensor<float>(1000, read_TG_shuntvoltage_callback); //drop across the shunt

  auto* ina219_TG_busvoltage = 
      new RepeatSensor<float>(1000, read_TG_busvoltage_callback);  //downstream voltage 

  auto* ina219_TG_loadvoltage = 
      new RepeatSensor<float>(1000, read_TG_loadvoltage_callback);   //battery voltage

  auto* ina219_TG_r2resistance = 
      new RepeatSensor<float>(1000, read_TG_r2resistance_callback);   //r2 resistance      

  // Send the temperature to the Signal K server as a Float

  ina219_TG_loadvoltage->connect_to(new SKOutputFloat("engine.coolant.Vin.voltage")); 

  ina219_TG_shuntvoltage->connect_to(new SKOutputFloat("engine.coolant.differential.voltage")); 

  ina219_TG_busvoltage->connect_to(new SKOutputFloat("engine.coolant.Vout.voltage")); 

  ina219_TG_r2resistance->connect_to(new SKOutputFloat("engine.coolant.R2.resistance")); 

ina219_TG_r2resistance
    ->connect_to(new TempInterpreter("/engine/coolant/curve"))
    ->connect_to(new Linear(1.0, 0.0, "/engine/coolant/calibrate"))
    ->connect_to(new SKOutputFloat("engine.coolant.temperature",
                                   "/engine/coolant/sk_path"));

  // To avoid garbage collecting all shared pointers created in setup(),
  // loop from here.
  while (true) {
    loop();
  }
}

void loop() { event_loop()->tick(); }