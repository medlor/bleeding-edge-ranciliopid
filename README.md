# ranciliopid - Open source PID for your espresso maschine

BLEEDING EDGE MASTER VERSION

Version 2.4.2

forked of the Rancilio-Silvia PID for Arduino described at http://rancilio-pid.de.

## Support / Contact
You can chat with us directly using our [discord server](https://discord.gg/VA5ZeacFdw).

## Most important features compared to rancilio-pid master:
1. New PID Controller "Multi-state PID with steadyPower (Bias)"
   - Auto-Tuning of all PID settings. No knowledge or special tunings required.
   - Distinct PID settings dependend on the current "state" of the maschine.
   - Most of the settings are either static or semi-automatically tuned, which does not require an PHD (German: Diplom) to understand.
   - Currently 5 states are implemented:
     - Coldstart (maschine is cold)
     - Coldstart stabilisation (directly after coldstart)
     - Inner Zone (temperature near setPoint)
     - Outer Zone (temperature outside of "inner zone")
     - Brewing
   - steadyPower is introduced which compensates the constant temperature loss due to environment
   - steadyPowerOffset is introduced which compensates the increased temperature loss when the maschine (brew head etc.) are still very cold.
   - PidController offers feature like I-value filtering, special handling of setPoint crossings and more (hard-coded)
   - PID Controller is now integral part of the software and not an external library.
1. Freely choose if you want the software to use WIFI, BLYNK and/or MQTT. Everythink can be enabled/disabled and stil have a flawlessly working PID controller.
1. Additionally if you want to to depend on a remotely running service (eg. blynk server on raspi), you can activate a MQTT-Server on the arduino itself!
1. Offline Modus is fixed and enhanced. If userConfig.h's FORCE_OFFLINE is enabled, then PID fully is working without networking. Ideal in situations when there is no connectivity or you dont want to rely on it.
1. Huge performance tunings and improvements under the hood which stabilizes the system (eg in situations of bad WIFI, hardware issues,..).
1. MQTT support to integrate maschine in smart-home solutions and to easier extract details for graphing/alerting.
1. Added RemoteDebug over telnet so that we dont need USB to debug/tune pid anymore (https://github.com/JoaoLopesF/RemoteDebug). While using OTA updates you can remotely debug and update the software!
1. "Brew Ready" Detection implemented, which detects when the temperature has stabilized at setPoint. It can send an
   MQTT event or have hardware pin 15 triggered (which can be used to turn a LED on).
1. All heater power relevant settings are now set and given in percent (and not absolute output) and therefore better to understand
1. Support for customizable item collections to beautify display output (thanks to helge!)
1. Safetly toogle added to shutdown heater on sensor malfunction (TEMPSENSORRECOVERY)
1. Many useful functions to be used internally getAverageTemperature(), pastTemperatureChange() + updateTemperatureHistory())

## ATTENTION:
- This software is tested thoroughly with the pid-only hardware solution on Silvia 5e, and with a permanently run full-hardware solution on an 10 year old Silvia. Also a 10 year old Gaggia Classic is tested successfully. I am grateful for any further feedback.
- Please monitor our maschine's temperature closely the first few run times. The muti-state pid controller should never lead to temperatures greater than 5 degress above setpoint!

## Sample bleeding-edge workings
- You can use the mqtt interface to export live data for monitoring purposes as can be seen in this [Grafana Dashboard](https://snapshot.raintank.io/dashboard/snapshot/lYe7XigrehSfVsvAWYifEwd2d5hNC0dl).

## Instructions on how to migrate from official rancilio to bleeding-edge
Installation is as explained on http://rancilio-pid.de/ but with following adapations:
1. Make screenshots of the official "Blynk App Dashboard" so that you can revert anytime.
1. Copy file userConfig.h.SAMPLE to userConfig.h and edit this file accordingly.
   - I propose to set MQTT_ENABLE=2 instead of blynk. But it is up to you.
     ```
     #define MQTT_ENABLE 2
     ```
1. Copy contents of folder ranciliopid\arduino-libs to your arduino sketchbook location (normally C:\Users\YOUR_NAME\Documents\Arduino)
   - Important: Open arduino IDE and configure the setting Tools->"lwip Variant" -> "1.4 High Bandwidth"
1. Flash and enjoy your espresso.
1. No tuning should be required normally. If you want/need to then use the method described below.

## Instructions on how to update to the latest version of bleeding-edge
1. Just overwrite all existing files with a newly released version.
2. Open your userConfig.h file, which had not been overwritten in previous step, and manually check (line by line!) that all updates to the new file userConfig.h.SAMPLE are reflected in your own userConfig.h.
3. Compile, upload and enjoy!

## Remote Control APIs
- MQTT
  - Instructions can be found at https://github.com/medlor/bleeding-edge-ranciliopid/wiki/MQTT-Setup
- Blynk
  - Instructions can be found at https://github.com/medlor/bleeding-edge-ranciliopid/wiki/Blynk-Setup

## Tunings instructions
- Instructions can be found at https://github.com/medlor/bleeding-edge-ranciliopid/wiki/PID-Tuning

## Debugging Howto
- Instructions can be found at https://github.com/medlor/bleeding-edge-ranciliopid/wiki/Debugging-Howto

## How to use a simple LED as brewReady signal
- Instructions can be found at https://github.com/medlor/bleeding-edge-ranciliopid/wiki/How-to-use-a-simple-LED-as-brewReady-signal

## Instructions on how to create new icon collections
- Instructions can be found at https://github.com/medlor/bleeding-edge-ranciliopid/wiki/Instructions-on-how-to-create-new-icon-collections

## Changelog
- 2.4.2 master:
  - Bugfix: Revert changes in regard to compiler warnings added in 2.4.1.
  - Bugfix (MQTT_ENABLE=1): Auto-Tuning settings are persisted across restarts.
  - Bugfix: steadyPowerOffsetTime is configurable via mqtt.
  - Bugfix: aggoTn is now correctly published via mqtt.
  - Added new circuit-diagrams (Thanks Tempcookies + Helge).
  - MQTT code refactorings.
- 2.4.1 master:
  - It fixes some compiler errors and warnings I was seeing on my system. (Thanks nils-werner)
  - It contains a small platformio.ini file for Platformio support. (Thanks nils-werner)
- 2.4.0 master:
  - Special winter theme added. Activate with ICON_COLLECTION=2 in userConfig.h.
  - Added support for a customizable screen-saver (#define ENABLE_SCREEN_SAVER)
  - Overwrite Version info at boot time via OVERWRITE_VERSION_DISPLAY_TEXT in userConfig.h.
  - Added power-off countdown to hint the user when the maschine is going to shutdown (ENABLE_POWER_OFF_COUNTDOWN) (Silvia 5E)
  - Power-off countdown starts 5min before shutdown.
  - Bugfix: Fix brew detection (OnlyPid)
  - Bugfix: End of BrewTimer correctly displayed (OnlyPid)
  - Bugfix: No power-off countdown when brewing.
  - Move display stuff into separate files
  - alpha code: Intitial support for an user-menu (ENABLE_USER_MENU/pinBrewButton)
  - alpha code: Initial infos about how to add a case with hardware-buttons to control basic settings directly at maschine.
- 2.3.0 master:
  - PID is completely auto-tuned and should work flawlessly after a few starts. No need to configure PID any longer.
  - Bleeding-Edge settings can be configured remotely without requiring a running service (blynk, mqtt-server) running in your network or internet. (No extra raspi required)
    You can now freely choose which service you want to use:
    1. Standalone without any network connectivity
    2. MQTT Server (runs on bleeding-edge arduino) to not depend on a remotely running service
    3. MQTT Client to connect to an mqtt-services (eg used by smart-home software)
  - MQTT fully implemented so you can publish and subscribe to configure settings and monitor states.
  - THANKS: Added Adrian's library to efficiently read TSIC values (https://github.com/lebuni/ZACwire-Library)
  - ATTENTION: New default values in userConfig.h.
    - It is recommended to intially use default values for STEADYPOWER, STEADYPOWER_OFFSET_TIME, STEADYPOWER_OFFSET, STARTTEMP, BREWDETECTION_POWER, BREWDETECTION_SENSITIVITY. Additionally BREWTIME is from now on also used in ONLYPID=1.
    - GPIO Pin Mapping moved to userConfig.h.
  - ATTENTION: Installation process changed. Libs must be installed as described in "Instructions on how to migrate from official rancilio to bleeding-edge
    - Copy contents of folder ranciliopid\arduino-libs to your arduino sketchbook location (normally C:\Users\YOUR_NAME\Documents\Arduino)
  - Improve PID:
    - Auto-tuning for starttemp is implemented. No need to adapt the STARTTEMP accordingly when SETPOINT is modified
    - steadyPowerOffset is gradually decreased over time to better compensate the warm up of maschine.
    - PID monitors and tunes steadyPowerOffset.
    - One time PID manipulation logic added.
    - PID's I parameter filter is reduced in certain situations more strictly.
  - BrewDetection optimized:
    - (ONLYPID=1) ATTENTION: The BREWTIME setting defines the time-frame of your "normal" brew (in seconds).
       When a brew is detected, the brew_timer starts and runs until BREWTIME is reached. During that time
       the heater is heating with BREWDETECTION_POWER.
    - BREWDETECTION_POWER behaviour changed:
      (ONLY_PID=0) The power is applied to heater during the complete brew process.
      (ONLY_PID=1) The power is applied until the brew_timer is larger than BREWTIME or the current temperature is 1.5 Celcius below setpoint.
    - (ONLYPID=1) BREWDETECTION_WAIT setting added: After a brew is started the software based BrewDetection is disabled for this number of seconds to prevent the detection of another brew when flushing water.
    - (ONLYPID=1) Software BrewDetection estimates the starttime of a brew, so that the brew counter in display is correct.
  - Debug Logs Improvements:
    - Add logs for brewReadyStatistic
    - When connecting by telnet/blynk the hardware- and software configuration is printed
  - Documentation Updates:
    - Enabled wiki at https://github.com/medlor/bleeding-edge-ranciliopid/wiki/
      - eg. Howto "Setup MQTT": https://github.com/medlor/bleeding-edge-ranciliopid/wiki/MQTT-Setup
    - Added link to sample daily Grafana dashboard.
  - Fix: userConfig.h: AUTH renamed to BLYNKAUTH
  - Fix: No starttemp tuning when when maschine is already warm.
  - Fix: burstPower working again.
  - Fix: If PID is manually disabled, heater utilization is correctly reported as 0%.
  - Fix: Heater overextending handling working as intended.
  - Fix: Potential crash when toogling pid on/off
  - Fix: Save pidON in eeprom.
  - MQTT_ENABLE=1: Configuration retained in mqtt topics are now used on startup.
  - Code cleanup and refactorings
  - Trigger brewReady when temperature is stable for 60sec (prev: 40s).
- 2.2.0_master:
  - Display functionality improved:
    - Replaced display lib Adafruit_SSD1306.h with U8G2. Direct support for SH1106_128X64 and SSD1306_128X64 via userConfig.
    - Completly new display widgets which show informations according to active PID state.
    - Support for icon collections to easily customize/share display widgets. Icon collection "simple" and "smiley" included.
    - Support for simple icon animations.
    - Service Status Icons are displayed if enabled in userConfig.
    - New DEFINES in userConfig.h. Update your config.
  - Improvement: Blynk On/Off button re-inits PID state similar to when machine power is turned on.
  - Fix: If DISABLE_SERVICES_ON_STARTUP_ERRORS=1 then WIFI reconnect attempts are also prevented.
  - Remove define EMERGENCY_TEXT.
  - Fix: PID State "brewing" is correctly detected when ONLYPID=0. (Thanks Helge)
- 2.1.0_master:
  - Networking:
    - Huge improvements in handling unstable WIFI networks and mqtt/blynk service unavailabilities.
    - You can disable/enable WIFI, MQTT or Blynk in userConfig.h and stil have a flawlessly working PID controller. Blynk is no longer an hard requirement!
    - Offline Modus is fixed and enhanced. If userConfig.h's FORCE_OFFLINE is enabled, then PID fully is working without networking. Ideal in situations when there is no connectivity or you dont want to rely on it.
    - Fix of EEPROM functionality: PID settings are correctly saved in EERPOM and correctly used if there are WIFI issues.
    - Instead of using dynamic IPs (over DHCPd) you have the option to set a static IP.
    - If blynk or mqtt is not working during startup, do not retry the connection periodically (configurable by userconfig.h DISABLE_SERVICES_ON_STARTUP_ERRORS)
  - New PID Variable "BREWDETECTION_POWER" introduced which defines the heater power during brewing.
  - Complete rewrite of "TSIC sensor read" based on the excellent ISR code by Adrian. (Thanks Adrian!)
    - Optimized "TSIC sensor read" to further increase performance.
    - Currently it takes <4ms to collect sensor data instead of the previous 78ms).
    - Fix: "sensor errors" do not occur anymore.
  - Performance/Stability:
    - Some system libs are optimized in performance and stability (src/ folder).
    - Remove all unneeded external libraries which are installed in system's arduino search path.
    - Code Tunings all over the place to increase performance and therefor stability.
    - Overall stability better by honoring critical processes.
  - Debuglogs can also be accessed via browser (see documentation).
  - PID calculations are moved from ISR to loop(). This improves stability even further.
  - Fix: Brew detection optimized.
  - Safetly feature: Done start brewing if the brew-button is switched "on" on startup
  - Fix: After power on, the 5 second wait time until heater starts is removed.
  - Library path adapted to support Arduino under Linux.
- 2.0.3_beta2:
  - Wifi disconnects handled better.
  - Implement blynk reconnect exponential backoff.
  - Set blynk reconnection timeout (3sec).
  - Added debug messages in ciritical paths.
- 2.0.2_master:
  - stable release
  - updated docs
- 2.0.2_beta2:
  - removed 8x8 display support
  - Code cleanup
- 2.0.2_beta1:
  - Major improvements in display related stuff (eg new icons,..) (thanks helge)
  - Restructure folders (thanks helge)
- 2.0.1_beta8:
  - ISR performance optimised when debug is active (spend time reduced from 0.8ms to 0.15ms).
  - Move DEBUGMODE to config.h.
- 2.0.1_beta7:
  - Emergency Logo can be replaced by a nice "milk steam" logo (EMERGENCY_ICON). (Thanks helge for the icon)
  - Regular status display is beautified and adapted to normal users needs. (Thanks helge for the code)
- 2.0.1_beta6:
  - Fix: Reducing "temperature sensor reading" errors.
  - Fix: steadyPowerOffset_Time is now correctly configurable via blynk.
  - Fix: Hardware switch sensitivity ("brewswitch") is increased from 1000 to 700.
  - Improved: mySteadyPower failure detections.
  - Code Cleanup + Refactorings.
- 2.0.1_beta5:
  - RemoteDebug inactivity time requires a version not yet in arduino library manager. Therefore a workaround is document in the meantime.
  - Pre-Infusion variables are now configurable in config.h: PREINFUSION, PREINFUSION_PAUSE, BREWTIME
  - Emergency temperature theshold is now configurable in config.h: EMERGENCY_TEMP
  - BrewDetection sensitivity is now configurable in config.h: BREWDETECTION_SENSITIVITY (renamed from brewboarder)
- 2.0.1_beta4:
  - Limit auto-tuning of steadyPower to reduce overly increased values.
  - Fix bug in brew().bezugsZeit calculation.
  - Renamed outputK to outputI.
  - Explanation of PID log line added.
- 2.0.1_beta3:
  - Pre-Infusion Values are now in seconds (previously milliseconds).  
    !! ATTENTION: Therefore you HAVE to change following values via blynk: brewtime, preinfusion, preinfusionpause !!
  - Reduce overhead of preinfusion functionality by having it run just every 50ms (and not every ms).
  - Add support for custom emergency text in display when temp >120 degree. (see EMERGENCY_TEXT in config.h.SAMPLE)
  - Emergency temperature threshold can now be set by variable emergency_temperature.
  - RemoteDebug inactivity time increased from 10min to 30min.
  - Add missing line-break in brew() debug logging.
- 2.0.1_beta2:
  - Better Installation Guide
  - Fix brew() function
  - Support virtual BrewReady LED in blynk
  - Improve pastChange() in Pid.compute(). (default Kd value has to be doubled)
  - Improve Auto-tuning.
  - Default PID Value Tunings
- 2.0.1_beta:
  - ATTENTION: EEPROM has changed. Therefore you have to connect to blynk at least once, and manually set correct settings in blynk app (see screenshots for default values).
  - New PID Controller "Multi-state PID with steadyPower (Bias)"
  - "Brew Ready" Detection implemented, which detects when the temperature has stabilized at setPoint. It can send an
    MQTT event or have hardware pin 15 triggered (which can be used to turn a LED on).
  - Because deadtime of Silvia5E is around 45seconds PID.compute() runs every 5 seconds (previous 1sec) and reduce header on/off switches by factor 5
  - Temperature polling is also now set to once every second (previous 400ms)
  - Code refactoring/cleanup and fixes of bugs.
  - BurstShot feature added to temporary overwrite PID controls (useful mainly for tests)
  - Added RemoteDebug over telnet so that we dont need USB to debug/tune pid anymore (https://github.com/JoaoLopesF/RemoteDebug)
    - Just "$ telnet rancilio_ip 23"
  - Fix: Recover EmergencyStop when temperature poll in setup() fails
  - EmergencyStop state logging improved
- 1.9.8h_alpha:
  - Feature: Implemented special 3 step cold-start mechanism. See special tuning instructions above.
- 1.9.8g:
  - Improvement: Broken temperature is detected when temp has increased more than >5 degrees (previous 25 degrees) in the last 0.4 seconds.
  - Improvement: Better brewReady detection by waiting for stable temperature within a longer time window (from 6 to 14secs).
  - TemperatureHistory increased from 6 seconds to 30seconds.
- 1.9.8f:
  - Added missing config.h.SAMPLE variables.
  - Added some more mqtt events.
- 1.9.8e:
  - Removed movAvg() due to several issues:
    - This is no moving avg, but something different (but it is working for "stable" temperature curves).
    - readIndex=0 is not used.
    - movingAvg does not compares currentValue with previousValue.
    - Faktor *100 makes no sense (?).
    - initializing methods/firstreading is refactored.
  - movAvg() is replaced by pastTemperatureChange() (+ updateTemperatureHistory())
  - Change order of brew-detection in main loop() to not confuse bPid.compute() due to flapping setTunings()
  - Feature: isBrewReady() to determine if temperature is stable to start brewing
  - Bugfix: Replace abs() with fabs().
- 1.9.8d:
- 1.9.8c:
  - Add more mqtt updates: kp,ki,kd, coldstart
  - Safe-guard: be sure pid internals are cleared when pid is dis/enabled.
- 1.9.8b:
  - Feature: Add support for MQTT.
  - Safeguard: HeaterPreventFlapping must never be > windowSize.
- 1.9.7:
  - Fix: If temperature sensor detects more than 150Celcius then it must be an error.
  - Fix: Typos / explanations.
  - Fix: Race condition when code is blocked for some time
  - Fix: Added initial temp poll to prevent error on start.
  - Feature: Detect missing temperature polls.
  - Feature: Add configurable BLYNKPORT.
  - Feature: Wifi hostname configuable.
  - New feature: Safe-guard: Stop heating forever if sensor is flapping!
  - Fix: Pid.Compute() is now in sync with isrCounter and is not loosing one tick each second. Heater flapping is reduced.


# Special Thanks
To the great work of the rancilio-pid.de team, just to mention a few: andreas, markus, toppo78, miau.  
Also to the nice people in the rancilio chat and the ones who contribute and give very much appreciated feedback like helge and Adrian!  

!! Thank you so much for the tasty cup of coffee I enjoy each day !!  


# Disclaimer
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
