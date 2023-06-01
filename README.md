
"C:\Users\Deepak Sharma\Documents\Arduino\libraries\mavlink\common\mavlink.h"


FQBN: arduino:avr:uno
Using board 'uno' from platform in folder: C:\Users\Deepak Sharma\AppData\Local\Arduino15\packages\arduino\hardware\avr\1.8.6
Using core 'arduino' from platform in folder: C:\Users\Deepak Sharma\AppData\Local\Arduino15\packages\arduino\hardware\avr\1.8.6

Detecting libraries used...
"C:\\Users\\Deepak Sharma\\AppData\\Local\\Arduino15\\packages\\arduino\\tools\\avr-gcc\\7.3.0-atmel3.6.1-arduino7/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -flto -w -x c++ -E -CC -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10607 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR "-IC:\\Users\\Deepak Sharma\\AppData\\Local\\Arduino15\\packages\\arduino\\hardware\\avr\\1.8.6\\cores\\arduino" "-IC:\\Users\\Deepak Sharma\\AppData\\Local\\Arduino15\\packages\\arduino\\hardware\\avr\\1.8.6\\variants\\standard" "C:\\Users\\Deepak Sharma\\AppData\\Local\\Temp\\arduino\\sketches\\4309104E3410AB94B26B76DB46AC5F8A\\sketch\\sketch_jun1a.ino.cpp" -o nul
Alternatives for mavlink.h: [mavlink]
ResolveLibrary(mavlink.h)
  -> candidates: [mavlink]
"C:\\Users\\Deepak Sharma\\AppData\\Local\\Arduino15\\packages\\arduino\\tools\\avr-gcc\\7.3.0-atmel3.6.1-arduino7/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -flto -w -x c++ -E -CC -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10607 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR "-IC:\\Users\\Deepak Sharma\\AppData\\Local\\Arduino15\\packages\\arduino\\hardware\\avr\\1.8.6\\cores\\arduino" "-IC:\\Users\\Deepak Sharma\\AppData\\Local\\Arduino15\\packages\\arduino\\hardware\\avr\\1.8.6\\variants\\standard" "-IC:\\Users\\Deepak Sharma\\Documents\\Arduino\\libraries\\mavlink" "C:\\Users\\Deepak Sharma\\AppData\\Local\\Temp\\arduino\\sketches\\4309104E3410AB94B26B76DB46AC5F8A\\sketch\\sketch_jun1a.ino.cpp" -o nul
Generating function prototypes...
"C:\\Users\\Deepak Sharma\\AppData\\Local\\Arduino15\\packages\\arduino\\tools\\avr-gcc\\7.3.0-atmel3.6.1-arduino7/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -flto -w -x c++ -E -CC -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10607 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR "-IC:\\Users\\Deepak Sharma\\AppData\\Local\\Arduino15\\packages\\arduino\\hardware\\avr\\1.8.6\\cores\\arduino" "-IC:\\Users\\Deepak Sharma\\AppData\\Local\\Arduino15\\packages\\arduino\\hardware\\avr\\1.8.6\\variants\\standard" "-IC:\\Users\\Deepak Sharma\\Documents\\Arduino\\libraries\\mavlink" "C:\\Users\\Deepak Sharma\\AppData\\Local\\Temp\\arduino\\sketches\\4309104E3410AB94B26B76DB46AC5F8A\\sketch\\sketch_jun1a.ino.cpp" -o "C:\\Users\\Deepak Sharma\\AppData\\Local\\Temp\\arduino\\sketches\\4309104E3410AB94B26B76DB46AC5F8A\\preproc\\ctags_target_for_gcc_minus_e.cpp"
"C:\\Users\\Deepak Sharma\\AppData\\Local\\Arduino15\\packages\\builtin\\tools\\ctags\\5.8-arduino11/ctags" -u --language-force=c++ -f - --c++-kinds=svpf --fields=KSTtzns --line-directives "C:\\Users\\Deepak Sharma\\AppData\\Local\\Temp\\arduino\\sketches\\4309104E3410AB94B26B76DB46AC5F8A\\preproc\\ctags_target_for_gcc_minus_e.cpp"
Compiling sketch...
"C:\\Users\\Deepak Sharma\\AppData\\Local\\Arduino15\\packages\\arduino\\tools\\avr-gcc\\7.3.0-atmel3.6.1-arduino7/bin/avr-g++" -c -g -Os -Wall -Wextra -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10607 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR "-IC:\\Users\\Deepak Sharma\\AppData\\Local\\Arduino15\\packages\\arduino\\hardware\\avr\\1.8.6\\cores\\arduino" "-IC:\\Users\\Deepak Sharma\\AppData\\Local\\Arduino15\\packages\\arduino\\hardware\\avr\\1.8.6\\variants\\standard" "-IC:\\Users\\Deepak Sharma\\Documents\\Arduino\\libraries\\mavlink" "C:\\Users\\Deepak Sharma\\AppData\\Local\\Temp\\arduino\\sketches\\4309104E3410AB94B26B76DB46AC5F8A\\sketch\\sketch_jun1a.ino.cpp" -o "C:\\Users\\Deepak Sharma\\AppData\\Local\\Temp\\arduino\\sketches\\4309104E3410AB94B26B76DB46AC5F8A\\sketch\\sketch_jun1a.ino.cpp.o"
C:\Users\Deepak Sharma\AppData\Local\Temp\.arduinoIDE-unsaved202351-4080-1c7hnk8.gd5c\sketch_jun1a\sketch_jun1a.ino: In function 'void loop()':
C:\Users\Deepak Sharma\AppData\Local\Temp\.arduinoIDE-unsaved202351-4080-1c7hnk8.gd5c\sketch_jun1a\sketch_jun1a.ino:30:63: error: 'MAV_DISTANCE_SENSOR_ULTRASOUND' was not declared in this scope
   mavlink_msg_distance_sensor_pack(1, 200, &msg, millis(), 1, MAV_DISTANCE_SENSOR_ULTRASOUND, 0, 0, 0, 0, 0, distance, 0, 0, 0, 0);
                                                               ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
C:\Users\Deepak Sharma\AppData\Local\Temp\.arduinoIDE-unsaved202351-4080-1c7hnk8.gd5c\sketch_jun1a\sketch_jun1a.ino:30:3: error: 'mavlink_msg_distance_sensor_pack' was not declared in this scope
   mavlink_msg_distance_sensor_pack(1, 200, &msg, millis(), 1, MAV_DISTANCE_SENSOR_ULTRASOUND, 0, 0, 0, 0, 0, distance, 0, 0, 0, 0);
   ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
C:\Users\Deepak Sharma\AppData\Local\Temp\.arduinoIDE-unsaved202351-4080-1c7hnk8.gd5c\sketch_jun1a\sketch_jun1a.ino:30:3: note: suggested alternative: 'mavlink_msg_data_stream_pack'
   mavlink_msg_distance_sensor_pack(1, 200, &msg, millis(), 1, MAV_DISTANCE_SENSOR_ULTRASOUND, 0, 0, 0, 0, 0, distance, 0, 0, 0, 0);
   ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   mavlink_msg_data_stream_pack

Using library mavlink in folder: C:\Users\Deepak Sharma\Documents\Arduino\libraries\mavlink (legacy)
exit status 1

Compilation error: 'MAV_DISTANCE_SENSOR_ULTRASOUND' was not declared in this scope
