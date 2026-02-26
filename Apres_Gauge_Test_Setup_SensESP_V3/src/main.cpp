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
#include "sensesp/signalk/signalk_output.h"
#include "sensesp_app_builder.h"
#include <Wire.h>  
#include <Adafruit_INA219.h>
#include "sensesp_onewire/onewire_temperature.h"
#include <Arduino.h>
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/curveinterpolator.h"

using namespace sensesp;
using namespace sensesp::onewire;

class TempInterpreter : public CurveInterpolator {
 public:
  TempInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the calculated Sender Resistance (R2) to the gauge Temperature (converted to Kelvin)
    clear_samples();
    // addSample(CurveInterpolator::Sample(knownOhm, knownTemperature-K)); - these numbers need changing
    add_sample(CurveInterpolator::Sample(1000, 200));
    add_sample(CurveInterpolator::Sample(147, 280));
    add_sample(CurveInterpolator::Sample(141, 290));
    add_sample(CurveInterpolator::Sample(135, 300));
    add_sample(CurveInterpolator::Sample(129, 310));
    add_sample(CurveInterpolator::Sample(123, 320));
    add_sample(CurveInterpolator::Sample(115, 329));
    add_sample(CurveInterpolator::Sample(105, 340));
    add_sample(CurveInterpolator::Sample(99, 346));
    add_sample(CurveInterpolator::Sample(0, 400)); 
  }
};
////////////INA219
  Adafruit_INA219 ina219_tg;
 
  const float Rgauge = 189; //this is the measured resistance of the gauge

  float read_TG_shuntvoltage_callback() { return (ina219_tg.getShuntVoltage_mV() / 1000);} //drop across the shunt (which is the gauge: Vin - Vout) - this is for debug 
  float read_TG_busvoltage_callback() { return (ina219_tg.getBusVoltage_V());} //downstream voltage to be supplied to the consumer (Vout) - this is for debug
  float read_TG_loadvoltage_callback() { return (ina219_tg.getBusVoltage_V() + (ina219_tg.getShuntVoltage_mV() / 1000));} //loadvoltage is the battery or supply voltage (Vin)
  float read_TG_current_callback() { return (ina219_tg.getCurrent_mA() / 1000);} //Current mA
  float read_TG_r2resistance_callback() { 
    float I_mA = ina219_tg.getCurrent_mA();
    if (I_mA <= 0.0001f) {
        return NAN;
    }
    float V_bus   = ina219_tg.getBusVoltage_V();
    float R_total = V_bus / (I_mA / 1000.0f);
    return R_total - Rgauge;
}
////////////INA219 end

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
                    ->enable_ota("thisisfine")
                    ->get_app();

/// 1-Wire Temp Sensor
  DallasTemperatureSensors* dts = new DallasTemperatureSensors(25); //digital 2

 // reference Temperature - this is for demonstration purposes to show how to use a 1-Wire sensor as a reference temperature for the gauge sender.
 // This is not needed for the gauge sender to work. You can remove this and the OneWireTemperature class if you don't need a reference temperature sensor.
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
      "environment.reference.temperature", "/Reference Temperature/skPath", "K");

  ConfigItem(ref_temp_sk_output)
      ->set_title("Reference Temperature Signal K Path")
      ->set_description("Signal K path for the Reference temperature")
      ->set_sort_order(300);

  ref_temp->connect_to(ref_temp_calibration)
      ->connect_to(ref_temp_sk_output);

/////INA219 start
  ina219_tg.begin();  // Initialize first board (default address 0x40)

  //// Temperature Sender Config ////
  auto* ina219_TG_shuntvoltage = 
      new RepeatSensor<float>(1000, read_TG_shuntvoltage_callback); //drop across the shunt

  auto* ina219_TG_busvoltage = 
      new RepeatSensor<float>(1000, read_TG_busvoltage_callback);  //downstream voltage 

  auto* ina219_TG_loadvoltage = 
      new RepeatSensor<float>(1000, read_TG_loadvoltage_callback);   //battery voltage

    auto* ina219_TG_current = 
      new RepeatSensor<float>(1000, read_TG_current_callback);   //battery voltage

  auto* ina219_TG_r2resistance = 
      new RepeatSensor<float>(1000, read_TG_r2resistance_callback);   //r2 resistance      

  // Send the temperature to the Signal K server as a Float
  ina219_TG_loadvoltage
  ->connect_to(new SKOutputFloat("engine.coolant.Vin.voltage",
                                 "/engine/coolant/Vin", "V"));
ina219_TG_shuntvoltage
  ->connect_to(new SKOutputFloat("engine.coolant.differential.voltage",
                                 "/engine/coolant/diffV", "V"));
ina219_TG_busvoltage
  ->connect_to(new SKOutputFloat("engine.coolant.Vout.voltage",
                                 "/engine/coolant/Vout", "V"));
ina219_TG_current
  ->connect_to(new SKOutputFloat("engine.coolant.current",
                                 "/engine/coolant/current", "A"));
ina219_TG_r2resistance
  ->connect_to(new SKOutputFloat("engine.coolant.R2.resistance",
                                 "/engine/coolant/R2", "Ohm"));

  ina219_TG_r2resistance  //take the R2 resistance
    ->connect_to(new TempInterpreter("/engine/coolant/curve"))   //Convert to temperature using the curve interpolator
    ->connect_to(new Linear(1.0, 0.0, "/engine/coolant/calibrate"))  //Calibrate the temperature if needed using a linear transform
    ->connect_to(new SKOutputFloat("engine.coolant.temperature",   //send the temperature to the Signal K server as a Float
                                   "/engine/coolant/sk_path", "K"));

  // To avoid garbage collecting all shared pointers created in setup(),
  // loop from here.
  while (true) {
    loop();
  }
}

void loop() { event_loop()->tick(); }