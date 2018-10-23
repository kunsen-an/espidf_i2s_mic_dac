# ESP32 I2S Microphone (ADMP441) and built-in DAC Example

This is based on ESP-IDF i2s_adc_dac Example.
Adapted to PlatformIO IDE for VSCode

Please refer to https://kunsen.net/ for details.

---

* This is an example of:
    * Recording sound from I2S Microphone (ADMP441) (24bit)
    * Replay the recorded sound via DAC
    * Play an audio file in flash
    
---

* Run this example
	* Set partition table to "partitions_adc_dac_example.csv" in menuconfig, or rename sdkconfig.default to sdkconfig directly.
	* Set IDF_PATH and run "make flash"
---

* This example will execute the following steps:
    1. Erase flash
    2. Record audio from I2S microphone  and save in flash
    3. Read flash and replay the sound via DAC
    4. Play an example audio file(file format: 8bit/8khz/single channel)
    5. Loop back to step 3
  
---
  
* Hardware connection:
	Please refer to https://kunsen.net


---

* How to generate audio files:
	
	* tools/generate_audio_file.py is an example of generate audio table from .wav files.
	* In this example, the wav file must be in 16k/16bit mono format.
	* generate_audio_file.py will bundle the wav files into a single table named audio_example_file.h
	* Since the ADC can only play 8-bit data, the script will scale each 16-bit value to a 8-bit value.
	* Since the ADC can only output positive value, the script will turn a signed value into an unsigned value.
	
---

* Note:
	* DAC can only play 8-bit data, so the wav file data are scaled to 8-bit data.
	* I2S DMA can only output 16-bit/32-bit data to DAC, DAC will only take the highest 8-bit data and output accordingly. 
	* Before I2S DMA can output data stream to DAC, the data format should be converted to 16-bit or 32-bit by padding zeros.



