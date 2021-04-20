/*
 Messwagen mit Bluetooth-Uebertragung basierend auf der Idee und dem Sketch von bubikopf und Matthias


 Revision History:
 ~~~~~~~~~~~~~~~~~
 13.02.21:  - Started based on the code from Matthias
            - Don't use an interrupt to detect the puses from the reed contact because
              - the interrupt pins are reserved used for DCC and the gravity sensor
              - it's not necessary since the pulses are slow
              - the interupt routine was to long => This would gnerate problems with DCC
            - New rpm calculation with a floating average filter
            - Read two analog channels
            - enable / disable the vacuum motor with "V" / "v"
            - Auto configute the bluetooth modul when the button on the BT-modul is pressed at power on
            - Possibility to program the Arduino over bluetooth
              Attention: The program can't be uploaded via the serial port if a program is connected via BT
              => The BT serial monitor has to be closed before the normal serial upload is used.
              The programming fails if the rail supply is missing and if the nano is only powered via USB.
            - Bluetooth and the normal RS232 port of the Nano mini Pro could be used in parallel.
              The signals are shown on both channels at the same time. Commands could be send also
              from both channels.
            - Always use the same send period (250ms) to be able to save the data with equal time stamps
            - Check the supply voltage and disable the BT modul it the voltage is to low
              It's enabled again if the voltage rises again. This is necessary because otherwise
              the BT module can't be used any more ;-(
            - Gradient measurement with tap detection
            - Debouncing is not nececcary for hall sensors (Checked with ossi)
            - Added DCC
 09.03.21:  - Using micros() to measure the speed => much more acurate measurement
              Checked with 1 kHz and a PWM period of 99.5%  (5400km/h)
              No lost pulses (Checked with pulse generator from picoscope)
              Max period @ 450km/h with d = 5.5mm: 84 Hz
                U = Pi*d
                Feq = Speed / U = 450km/h / (87 * Pi * 5.5 mm) * 1h / 3600 s * 1000000 mm / km = 83.15 Hz
                10 Hz => Speed = 54.1 km/h
              Lux Vacuum: D = 11mm, 2 magnets => d = 5.5 mm
            - Using micros() for the debug loop timer
            - Added other counters
              - Vacuum distance and time
 13.03.21:  - Changed the font (Minimal font with only numbers and icons) to save 7% FLASH
 19.04.21:  - Control the speed of the vacuum motor via DCC FN7
            - Flip the display by 180∞ with DCC FN9
            - New symbol for the internal voltage


 Ge‰nderte Pins gegen¸ber meinem Lochrasteraufbau
 Arduino  Alt                          Neu
 D4      *TAP_ARDU_PIN Int1 G-Senor   *PullUp Reset Pin        Geht Micht => SJ1 verbinden
 D5      *Enable Vakuum               *BLUETOOTH_RESETLOCK_PIN
 D6      *BLUETOOTH_EN_PIN            *LED
 D7      *BLUETOOTH_DISABLE_PIN        Hall2
 D9      *BLUETOOTH_RESETLOCK_PIN     *Enable Vakuum
 D10      nc                          *TAP_ARDU_PIN Int1 G-Sensor
 D11      nc                           Servo
 D12      nc (A6)                     *BLUETOOTH_DISABLE_PIN
 A0      *RAIL_VOLT_PIN               *Ext_VOLT_PIN 5-7V Mess
 A1      *Ext_VOLT_PIN 5-7V Mess      *BLUETOOTH_EN_PIN
 A2       nc (A4=SDA)                  Servo FET
 A3       nc (A5=SCL)                 *RAIL_VOLT_PIN
 A7       nc                           Taster


 SJ1 Muss Verbunden werden

Achtung: Es ist kein Elko in der Versorgungsspannung => Probleme bei DCC RailCom Austastl¸cke wenn Vakuum Motor l‰uft

G-Sensor Fixieren oder gleich Festlˆten

Achtung: Bei SJ3 f¸hrt der SuperCap Mess Min in die Mitte und der Widerstand ist auﬂen

Testen:
 - DCC                              Ok
 - Sauger                           Ok Kommt mir st‰rker/lauter vor => 4 Stupen per PWM eingbaut
 - BT                               Ok => RX und TX mussten getauscht werden
 - Hall Sensor                      Ok => der auf der Platine vorgesehene Sensor ist nicht empindlich genug und hat die falche Magnetfeldrichtung
 - Saving the EEPROM @ Low Voltage  OK


 Inbetriebnahme des Bluetooth Moduls
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 Nachdem die Software auf dem Arduino installiert wurde muss das BT-Modul konfiguriert werden. Dazu h‰lt man
 die Taste am BT-Modul fest w‰hrend man die Versorgungsspannung des Messwagens anschaltet. Die Taste muss
 5 Sekunden lang gehalten werden. Es reicht nicht, wenn man anstelle des Trennens der Versorgungsspannung nur
 den Reset Knopf des Arduinos bet‰tigt.

 Achtung: Evtl. ist der Schrumpfschlauch ¸ber das BT-Modul zu steif zum zuverl‰ssigen bet‰tigen des Tasters.

 Die Konfiguration erfolgt automatisch. Das BT-Modul bekommt in diesem Schritt den Namen "Messwagen?"
 zugewiesen (Siehe #define BT_NAME).

 W‰hrend der Konfiguration Blinkt die LED auf den Arduino ein paar Mal unregelm‰ﬂig kurz.

 Wenn alles gut gegangen ist, dann blinkt sie am Ende 5-mal langsam (0.5Hz).
 Im Fehlerfall blinkt sie 20-mal ganz schnell.

 Koppeln mit dem Rechner
 ~~~~~~~~~~~~~~~~~~~~~~~
 Anschlieﬂend muss das BT-Modul mit dem Windows Rechner (oder dem Smartphone) gekoppelt werden.
 Hier die Beschreibung f¸r Windows 10:
 In den Windows Einstellungen (Tastenkombination WIN+I) geht man zu "Ger‰te Bluetooth, Drucker, Maus".
 Zun‰chst pr¸ft man ob der "Messwagen?" evtl. schon von fr¸heren Versuchen gekoppelt ist. Dazu scrollt man
 auf der Seite nach unten. Hier kann man eine bestehende Koppelung auch wieder trennen.

 Wenn der Messwagen noch nicht verbunden ist, dann klickt man ganz oben auf "Bluetooth- oder anders
 Ger‰t hinzuf¸gen" und Anschlieﬂend "Bluetooth M‰use, ...".
 Dann sollte nach einiger Zeit der Messwagen? Auftauchen, wenn er nicht bereits verbunden war. Mit einem
 Klick auf den Namen kommt man zur Password Eingabe. Standardm‰ﬂig wird "1234" verwendet.
 Damit sollte der Messwagen in der Liste der gekoppelten Ger‰te erscheinen.

 Jetzt muss man noch herausfinden welchen COM Port Windows f¸r den Messwagen reserviert hat. Dazu klickt man
 ganz unten in dem "Bluetooth- und andere Ger‰te" Fenster auf "Weitere Bluetooth-Optionen".
 In dem erscheinenden Fenster aktiviert man den Tab "COM-Anschl¸sse".
 Dort sollte der "Messwagen?" zwei Mal erscheinen. Der erste COM Port mit der Richtung "Ausgehend" ist der
 Port ¸ber den der Wagen angesprochen werden kann. Er wird in der Arduino IDE Gew‰hlt, wenn man die Daten
 sehen will oder wenn man das Programm per Bluetooth zum Arduino schicken will.

 Diese Schritte m¸ssen nur einmal gemacht werden.

 Dummerweise gibt es unterschiedliche BT Module und auch bei optisch genau dem gleichen Modul kann eine andere
 Software installiert sein. Bei dem von mir verwendeten Modul ist die Version 4.0 installiert. Das kann man
 ¸ber AT Kommandos ermitteln. Mit dieser Software funktioniert das invertieren des Status Signals nicht mehr.
 Bei der ‰lteren Version 2.0 geht es laut angaben im Netz noch. Darum habe ich bei der Schaltung einen
 Transistor vorgesehen mit dem das Signal invertiert wird. Damit sollte es mit beiden Softwarest‰nden zu
 benutzen sein.


 Achtung: wenn man das Modul per DCC Versorgt, dann kann es sein, dass der Reset Impuls zum Programmieren
 per USB nicht durchkommt. Dann kann man das Hochladen manuell starten indem man den Reset Button am
 Arduino dr¸ckt. Alternativ l‰sst man die +Leitung des Programmiersteckers weg.

 HC-05
 ~~~~~
 Status LED:
 - Schnelles Blinken:            Warten auf Verbindung
 - Langsames doppeltes Blitzen:  Verbunden
 - Langsames Blinken:            Configutationsmode f¸r AT Befehle. Taste beim Einschalten Dr¸cken

 AT Befehle:
 - Beim Einschalten Taste am Modul dr¸cken
 - Baud 38400
 - Achtung "Sowohl NL als auch CR" muss aktiv sein
 - Erster Test: AT  => OK
 - Es gibt anscheinend zwei AT Kommando Modes. Diese werden ¸ber den EN Pin bzw. den Taster aktiviert.
   Der Taster verbindet bei meinem Modul den EN Pin mit 3.3V.
   (Siehe: Seite 2: "HC-0305 serail module AT commamd set 201104 revised_2.pdf")
     1. Wenn beim Einschalten der Versorgungsspannung der Taster Bet‰tigt ist oder EN auf 3.3V liegt,
        dann Startet das Modul im AT Modus. Dabei wird die Baudrate 38400 gesetzt.
     2. Der EN Pin wird nach dem Anlegen der Versorgunsspannung an 3.3V geklemmt.
        Dann kˆnnen AT Kommandos mit der eingestellten Baudrate gesendet werden. Standardm‰ﬂig
        ist die Baudrate 9600. Sie kann mit dem AT Befehl "AT+UART=57600,0,0" ge‰ndert werden
        Wenn der Enable Pin auf 0 V gelegt wird, dann ist der AT Kommando Modus wieder beendet.
     Hier steht, dass es noch eine zweiten AT Modus gibt den man aktiviert wenn man den Enable Pin
     Dauerhaft aktiviert: https://wolles-elektronikkiste.de/hc-05-und-hc-06-bluetooth-module


 - Befehle
   - AT+ORGL             Default Einstellungen (Achtung: Generiert Reset am Arduino)
   - AT+UART=57600,0,0
   - AT+UART?
   - AT+NAME=MESSWAGEN1
   - AT+NAME?
   - AT+POLAR=1,0         Geht nicht
                          Damit kann man bei einer ‰lteren SW Version die Polarit‰t der Ausgabepins invertieren.
                          Hier wird auch davon Berichtet: https://arduino.stackexchange.com/questions/71916/arduino-bluetooth-upload-problem-hc-05-v4-0
                          Fake HC-05? USB Port zu schwach? : https://forum.arduino.cc/index.php?topic=645772.0

 Upload per Bluetooth:
 - Das BT-Modul muss so konfiguriert werden, dass es die gleiche Baudrate verwendet wie avrdude.
   Beim Nano Mini Pro ist das 57600. Mit dem folgenden AT Befehl kann die Baudrate eingestellt werden:
     AT+UART=57600,0,0
 - Der Statusausgang muss ein Reset des Arduinos auslˆsen. Der Resetimpuls f¸r den Arduino muss negativ sein.
   Dummerweise ist der Statusausgang normalerweise High Aktiv. Laut Internet kann man das durch
   umprogrammieren des BT-Moduls ‰ndern. Das geht mit dem AT Befehl:
     AT+POLAR=1,0
   Das Funktioniert aber nicht bei meinem Modul. Von dem Problem wird auch hier berichtet:
   https://arduino.stackexchange.com/questions/71916/arduino-bluetooth-upload-problem-hc-05-v4-0
   => Ich verwende einen FET mit dem das Signal invertiert wird. Dadurch hat man auch kein Problem
   mit damit, dass der Status Ausgang einen Pegel von 3.3V hat.

                                      +5V
                                       |
                                      |1|           Arduino
                                      |k|       .-----------
                               BS170   |  220nF |
                                 ||----*---||---|  Reset
                                 ||<-.          |
                             .---||--|          |
                             |      _|_         |
   .---------------------.   |                  |
   |               State |---*-[10K]-|          |
   |                 RXD |-----------[1K]-------| TX
   |                 TXD |-----------[1K]--*----| RX
   |                 GND |---GND           |    |
   |                 VCC |---VCC          |2|   |
   |                  EN |                |K|   '-----------
   '---------------------'                |2|
                                          _|_


   Update the schematic


   Rail+
   -----[82K]--*------  A0        20V        1.1V
               |               ---------  =  ----     =>   R1 = R2 * 20V * (1/1.1V - 1/20V)
              |4|               R1 + R2       R2
              |K|
              |7|
              _|_

  Ad_Val/5 [0.1V]

  Hall Sensor: TLE4905


  Das Bluetooth Modul steigt bei einer Spannung von 4.2V aus und startet nicht wieder wenn die
  Versorgungsspannung wieder grˆﬂer wird ;-(
  Darum kann ¸ber Transistor Q3 welche von Pin D7 geschaltet wird die Versorgungsspannung
  des Bluetooth Moduls abgeschaltet werden. Beim Wiedereinschalten wird aber ¸ber den State
  Ausgang des BT-Moduls ein Reset des Arduinos generiert. Das wird zum Flashen des Arduinos
  ¸ber BT benˆtigt. Bei einen Neustart wegen Unterspannung ist das aber nicht gew¸nscht.
  ‹ber Transistor Q1 und Pin D8 kann das verhindert werden.
  Nach dem Neustart des BT-Moduls muss sich das Handy wieder mit dem Messwagen verbinden.
  Dabei wird aber auch ein Reset des Arduinos ausgelˆst. Auch das kann ¸ber Q1 verhindert werden.

 Links:
 - Bluetooth:
   https://wolles-elektronikkiste.de/hc-05-und-hc-06-bluetooth-module
 - ADXL345 Beschleunigungssensor
   https://wolles-elektronikkiste.de/adxl345-teil-1

  Umrechnung der Impulse
  Umfang = Pi * d
  Max Speed 400 km/h 1:1
  => 1:87 = 400/3.6*1000/87 = 1277 mm/s
  ==> Bei einem Raddurchmesser von 11mm und zwei Magneten (Lux Staubsauger)
      kommen bei 400km/h 74 Impulse pro Sekunde


  ToDo:
  - Sauger Status Bit anzeigen (Schalte manchmal aus und geht nich twieder an)
  - BT Reichweite testen bei dem Puzzle Einbau
  - Support f¸r das BT-Modul "BEI-09 Android IOS BLE 4,0"
  - Reset per BT Abschaltbar machen. Das Braucht man ja nur zum Programmieren per BT
  Sp‰ter:
  - Startwert der Zeitmessungen auf 500 ms zur vermeidung von Rundungsfehlern
  - Defines auf #ifndef umstellen und config datei
  - FormatString mit dem bestimmt werden kann welche Messwerte im serial Plotter gezeigt werden
  - Update the schematic im Prog.

  - Skallierte Ausgabe auf seriellem Monitor km/h / m ? => Nein

  - OLED Lˆschen wenn 5V weg gehen
  - Pr¸fen ob man beim HC-05 ein Software Update machen kann
  - Messung der bluetooth St‰rke (RSSI)
    https://electronics.stackexchange.com/questions/98160/how-to-get-rssi-of-bluetooth-specfically-hc-05
  - Programm aufr‰umen
  - BrownOut Detection aktivieren
  - Das Wiedereinschalten des BT-Moduls klappt nicht immer wenn    => Solte jetzt passen
    die externe Spannung weggefallen ist ;-(



 Print Memory Usage:
 ~~~~~~~~~~~~~~~~~~~
 - Nach dieser Anleitung geht es: https://arduino.stackexchange.com/questions/31190/detailed-analyse-of-memory-usage
     cd C:\Program Files (x86)\Arduino\hardware\tools\avr\bin
     set elf_File=C:\Users\Hardi\AppData\Local\Temp\arduino_build_601332\Messwagen_BT2.ino.elf

     RAM:
       avr-nm.exe -Crtd --size-sort "%elf_File%" | C:\Users\Hardi\AppData\Local\atom\app-1.28.0\resources\app.asar.unpacked\node_modules\dugite\git\usr\bin\grep.exe -i ' [dbv] ' | sort
     FLASH:
       avr-nm.exe -Crtd --size-sort "%elf_File%" | C:\Users\Hardi\AppData\Local\atom\app-1.28.0\resources\app.asar.unpacked\node_modules\dugite\git\usr\bin\grep.exe -i ' [tw] ' | sort


   Attention: "printf" or "sprintf" uses a lot of memory:
     00000948 T vfprintf  = 2376 Byte
*/
#include <Arduino.h>
#include <EEPROM.h>
#include "Messwagen_BT2_Config.h"

#ifndef BT_NAME
#define BT_NAME                  "Messwagen1" // Name of the device shown on bluetooth
#endif

#ifndef DEF_PERIMETER
#define DEF_PERIMETER               0.0172788 // perimeter (Umfang) [m]
#endif
#ifndef DEF_SCALEFACT
#define DEF_SCALEFACT                      87 // H0: 1/87  1/22.5 Industriebahn
#endif

#define VERSTR                          "0.1" // Only numbers
#define EE_VERSION_STR        "Messw" VERSTR

// Configuration
#ifndef GRADIENT_MEASUREMENT
#define GRADIENT_MEASUREMENT                1 // Measure the gradient using a ADXL345 chip
#endif
#ifndef ENABLE_TAP_DETECTION
#define ENABLE_TAP_DETECTION                1 // Activate the tap detection to switch the displays and enable the vacuum motor
#endif
#ifndef TAP_THRESHOLD
#define TAP_THRESHOLD                     2.0 // Threshold (float) [g] to activate special functions with single / double tab (0 to disable, Range 2..8 3:Recommended)
#endif
#ifndef DOUBLE_TAB_TIME_WINDOW
#define DOUBLE_TAB_TIME_WINDOW            318 // Time to detect a double tab (Max 318 ms)
#endif

#ifndef OLED_DISP
#define OLED_DISP                          96 // OLED Types (0:Disabled, 87:0.87", 91:0.91", 96:0.96", 13:1.3")
#endif
#ifndef OLED_CONTRAST                         // Set the contrast or brightness for the display (if supported).
#define OLED_CONTRAST                      -1 // Range: 0 (no contrast) to 255 (maximum contrast or brightness)
#endif                                        // -1 = Use standard (and save 22 bytes FLASH)
#ifndef ENABLE_DISPLAY_ROTATION
#define ENABLE_DISPLAY_ROTATION             1 // Enable the rotation of the display using DCC
#endif
#ifndef RESET_TRIP_IF_STOPED
#define RESET_TRIP_IF_STOPED             5000 // Reset the trip counter if stopped for the given time in ms. Also used to store the EEPROM
#endif

#ifndef DCC_ADDR
#define DCC_ADDR                          601 // Set to -1 to disable DCC. (Extended DCC addresses up to 10239 are supported)
#endif
#ifndef ENAB_CV_ADR_CHANGE                    // If activated the DCC address could be changed by writing to CV1, CV17, CV18 and CV29
#define ENAB_CV_ADR_CHANGE                  1 // Attention: The Controller program will generate an error because there is no hardware
#endif                                        // on the main board to generate the acknowledge signal. But the address is written any how

#ifndef MIN_VOLTAGE_LINE
#define MIN_VOLTAGE_LINE                   15 // Constant line to be able to quickly see if the voltage drops below a certain level
#endif

#ifndef ENABL_BT_LOW_VOLT_DETECTION
#define ENABL_BT_LOW_VOLT_DETECTION         0 // Detect bluetooth low voltage and disable/enable the BT module. Useful if a supercap without stepUp is used.
#endif
#ifndef BT_VOLT_LOW
#define BT_VOLT_LOW                      4350 // [mv] If the voltage go's below this value the bluetooth module is disabled.
#endif                                        // In addition the EEPROM is saved. This is also active if ENABL_BT_LOW_VOLT_DETECTION is 0
#ifndef BT_VOLT_NORM
#define BT_VOLT_NORM                     5000 // [mv] The bluetooth module is enabled again
#endif

#ifndef VACU_SUPERCAP_VOLT_CONTR
#define VACU_SUPERCAP_VOLT_CONTR            1 // Disable the vacuum motor if the voltage of the supercap drops below VACU_DISAB_VOLT
#endif
#ifndef VACU_ENAB_VOLT
#define VACU_ENAB_VOLT                   5400 // [mv] Enable the vacuum motor if the supercap voltage is above this voltage
#endif
#ifndef VACU_DISAB_VOLT
#define VACU_DISAB_VOLT                  5300 // [mV] Disable the vacuum motor if the supercap voltage is below this voltage
#endif

#ifndef SERIAL_PRINT_VALUES
#define SERIAL_PRINT_VALUES                 1 // Set to 0 to disable the serial output of the values for debugging
#endif
#ifndef DEBUG_MAINLOOP_CHECK
#define DEBUG_MAINLOOP_CHECK                0 // Check the main loop time (Debug)
#endif
#ifndef DEBUG_HALL
#define DEBUG_HALL                          0 // Check lost hall pulses. In this case the detected speed drops to the half value
#endif

// Pin Numbers
#ifndef LED_PIN
#define LED_PIN                            13 // Status LED
#endif
#ifndef HALLPIN
#define HALLPIN                             3 // Hall sensor pin number (must be an interrupt pin (2 or 3))
#endif
#ifndef RESET_PULLUP
#define RESET_PULLUP                        0 // Reset Pullup Pin (Attention SJ1 must be open)
#endif
#ifndef TAP_ARDU_PIN
#define TAP_ARDU_PIN                       10 // pin for ADXL345 tap signal (Connected to INT1 at the ADXL345)
#endif
#ifndef VACUUM_ENAB_PIN
#define VACUUM_ENAB_PIN                     9
#endif
#ifndef Ext_VOLT_PIN
#define Ext_VOLT_PIN                       A0 // External Voltage measurement (Supercap voltage in pulling Lok)
#endif
#ifndef BLUETOOTH_EN_PIN
#define BLUETOOTH_EN_PIN                   A1 // Enable pin for the HC05 module
#endif
#ifndef RAIL_VOLT_PIN
#define RAIL_VOLT_PIN                      A3 // Rail voltage measurement
#endif
#ifndef INT_VOLT_PIN
#define INT_VOLT_PIN                       A6 // Internal 5V measurement
#endif
#ifndef BLUETOOTH_DISABLE_PIN
#define BLUETOOTH_DISABLE_PIN              12 // Disable pin for the BT power
#endif
#ifndef BLUETOOTH_STATE_PIN
#define BLUETOOTH_STATE_PIN                 8
#endif
#ifndef BLUETOOTH_RESETLOCK_PIN
#define BLUETOOTH_RESETLOCK_PIN             5 // Switch to +5V to disable the BT Reset. Used as open collector
#endif
#ifndef ZERO_MARK_HALL_PIN
#define ZERO_MARK_HALL_PIN                  0 // Additional hall switch to detect the position on the track (Set to 0 to disable)
#endif
#ifndef LIGHT_PIN
#define LIGHT_PIN                           6 // light which could be switched by DCC or bluetooth (Set to 0 to disable)
#endif

#ifndef SHOW_INTERVAL
#define SHOW_INTERVAL                     250 // Interval for updating the data on the serial port and bluetooth in ms
#endif
#ifndef VACUUM_ON_TIME
#define VACUUM_ON_TIME                   4000 // Duration [ms] how long the vacuum motor is enabled if a hall impulse is detected
#endif
#ifndef MAX_PERIOD                            // Maximal period period between two hall pulses. This constant defines the minimal
#define MAX_PERIOD                       4000 // detected speed. But it also defines the time to ramp down if the train is stopped
#endif                                        // The VACUUM_ON_TIME should be greater or equal to prevent turning on and of at slow speeds
#ifndef PWM_VACUUM_CONTROL
#define PWM_VACUUM_CONTROL                  1 // Control the speed of the vacuum motor by PWM
#endif

#ifndef EE_VER_NR
#define EE_VER_NR                           0 // Change if the EEPROM layout has been changed
#endif

// Internal Defines

#define EE_CHECK_SIZE sizeof(EE_VERSION_STR) + 1 + 2 // Version string + Ver_Nr + DCC_Addr_Def
#define EE_START_ADDR                       0 // First used EEPROM Adress

#define OLED_TOTAL_LINES                   15 // Number of lines in the switch() statement in Update_OLED()

#define S_KMH "\41"  // kmh string small
#define S_m   "\42"  // m   string small
#define S_DEG "\43"  // ∞   Degree string
#define S_km  "\44"  // km  string small
#define S_PCT "\45"  // %
#define S_h   "\46"  // h   string small
#define S_s   "\47"  // s   string small
#define S_V   "\50"  // V   string small
#define S_GRD "\75"  // Gradient icon
#define S_TLD "\76"  // Tild icon
#define S_MAX "\111" // Max Speed
#define First_ICON   59


#define USE_MICROS 1
#if USE_MICROS
  #define SPEEDTIMER micros
  #define RPS_FACT                   1000000L
  #define DURATION_FACT                 1000L
#else // Old
  #define SPEEDTIMER millis
  #define RPS_FACT                      1000L
  #define DURATION_FACT                    1L
#endif

#if GRADIENT_MEASUREMENT
  #include <Wire.h>
  #include <ADXL345_WE.h>

  #define ADXL345_I2CADDR 0x53 // 0x1D if SDO = HIGH
  ADXL345_WE myAcc(ADXL345_I2CADDR);
  volatile bool tap = false;
#endif

#if OLED_DISP > 0
  #include <Arduino.h>
  #include <U8x8lib.h> // Vers. 2.28.10  braucht 102 bytes weniger Speicher als Ver. 2.27.6

  // Own font which contains only numbers and some special characters (Icons) to save memory
  // Normal font with letters and numbers 3333 byte
  // Special font 1316 byte
  #define U8G2_FONT_SECTION(name) U8X8_FONT_SECTION(name)
  #define U8X8_FONT_SECTION(name) U8X8_SECTION(".progmem." name)
  #define u8x8_pgm_read(adr) pgm_read_byte_near(adr)
  #define U8X8_PROGMEM PROGMEM
  #include "u8x8_font_px437wyse700a_2x2_Mini.h"
  #define FONT_NAME u8x8_font_px437wyse700a_2x2_Mini

  // If the data should be shown simultanusly on two displays the second display must be connected to the
  // Arduino with a diode in the SDA line (OLED Display ---|>|--- Nano)

  // Supported OLED Displays:
  // 0.87" I2C 128x32  OLED  SSD1316  (https://de.aliexpress.com/item/4000182887362.html)
  // 0.91" I2C 128x32  OLED  SSD1306  (https://de.aliexpress.com/item/4001028654247.html)
  // 0.96" I2C 128X64  OLED  SSD1306  (https://de.aliexpress.com/item/32643950109.html)
  // 1.3"  I2C 128X64  OLED  SH1106   (https://de.aliexpress.com/item/32683739839.html)

  #if OLED_DISP == 87
       U8X8_SSD1316_128X32_HW_I2C u8x8(U8X8_PIN_NONE);                     // 0.87" Display
       #define OLED_Lines 2
  #endif
  #if OLED_DISP == 91
       U8X8_SSD1306_128X32_UNIVISION_HW_I2C u8x8(U8X8_PIN_NONE, SCL, SDA);
       #define OLED_Lines 2
  #endif
  #if OLED_DISP == 96
       U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE, SCL, SDA);    // 0.96" Display with 128x64 pixel
       #define OLED_Lines 4
  #endif
  #if OLED_DISP == 13
       U8X8_SH1106_128X64_NONAME_HW_I2C  u8x8(U8X8_PIN_NONE, SCL, SDA);    // 1.3"  Display with 128x64 pixel
       #define OLED_Lines 4
  #endif

  #define START_SCR_DURATION 2000 // Init with the time to display the start screen
  uint32_t UpdateOLED = START_SCR_DURATION;
#endif // OLED_DISP > 0


#if DCC_ADDR >= 0
  #include <NmraDcc.h>
  NmraDcc  Dcc;
#endif // DCC_ADDR


uint8_t  Enable_Distance_Measurement = 2;
uint32_t Hall_Cnt = 0;                // Distance counter   (Display Test: 578743 / 57874331)

uint32_t showTime;                    // Time stamp when the display was updated the last time
uint32_t curTime;
uint32_t Check_Supply_Volt_t = 1000;  // It takes a while until the voltage readings are stable after power on
uint32_t U0;                          // Rail voltage
uint32_t U1;                          // External voltage
uint32_t U6;                          // Arduino voltage
float    Filt_Gradient = 0;
float    Filt_Tilt     = 0;

#define  BT_OFF        0
#define  BT_STARTING   1
#define  BT_NORMAL     2
uint8_t  BT_State = BT_NORMAL;        // bluetooth state

uint32_t TurnOff_Vacuum = 0;          // Time to turn off the vacuum motor

#if RESET_TRIP_IF_STOPED
  uint32_t Stopped_t = 0;
  uint32_t Cnt_Stopped = 0;
#endif

typedef struct
    {
    char     Ver[sizeof(EE_VERSION_STR)];
    uint8_t  Ver_Nr;
    uint16_t DCC_Addr_Def;
    uint8_t  OffsetCorrected;
    float    AngleOffsetX;
    float    AngleOffsetZ;
    uint8_t  Vacuum;
    float    ScaleFact;
    float    Perimeter;
    uint8_t  ActOLEDDisp;
    uint16_t DCC_Adress;
    uint32_t Vacu_Cnt;
    uint32_t Vacu_Time;
    uint32_t T_Vacu_Cnt;
    uint32_t T_Vacu_Time;
    uint32_t Total_Cnt;
    uint32_t Total_Time;
    uint8_t  Light;
    uint8_t  VacuumSpeed;
    uint8_t  FlipMode;
    } EE_Data_T;
EE_Data_T ee;

uint32_t LastImpTime                = 0;
float    rps                        = 0;
float    rps_filt                   = 0;
uint32_t rps_Period                 = 0;
float    SpeedScale;
float    rps_max                    = 0;


volatile uint8_t  Hall_Imp_Detected = 0;
volatile uint32_t Hall_Imp_Time;

uint8_t ZMark = 0;

#if DEBUG_HALL
  float    Min_rps                  = 9999L;  // Debug
  uint16_t Multi_Hall_Pulses        = 0;
#endif

bool    Vacuum_State                = 0;

#if PWM_VACUUM_CONTROL
  #include "PWMFreak.h"
#endif

#if VACU_SUPERCAP_VOLT_CONTR
  bool Vacuum_Voltage_Ok = 0;
#endif


//------------------
void Hall_Int_Proc()
//------------------
// Is called by an interrupt on the falling edge of the hall signal
{
  Hall_Imp_Time = SPEEDTIMER();
  ++Hall_Imp_Detected;          // In case Check_Hall() is not called between two interrupts
}

//---------------
void Check_Hall()
//---------------
// Is called in the main loop
{
  static bool First_Hall = 1;
  if (Hall_Imp_Detected)
       {
       uint8_t New_Cnt = 0;
       noInterrupts();
       if (!First_Hall)
          {
          rps_Period  = Hall_Imp_Time - LastImpTime;
          LastImpTime = Hall_Imp_Time;
          if (rps_Period) rps = (float)(RPS_FACT * Hall_Imp_Detected) / rps_Period;
          New_Cnt = Hall_Imp_Detected;
          }
       #if DEBUG_HALL // Debug
         if (Hall_Imp_Detected > 1) Multi_Hall_Pulses += Hall_Imp_Detected -1;
       #endif
       Hall_Imp_Detected = 0;
       interrupts();

       First_Hall = 0;

       if (Enable_Distance_Measurement)
          Hall_Cnt += New_Cnt;

       ee.Total_Cnt += New_Cnt;

       if (ee.Vacuum)
          {
          ee.Vacu_Cnt   += New_Cnt;
          ee.T_Vacu_Cnt += New_Cnt;
          if (Vacuum_State==0)
             {
             Vacuum_State = 1;
             #if PWM_VACUUM_CONTROL
                 analogWrite(VACUUM_ENAB_PIN, ee.VacuumSpeed);
             #else
                 digitalWrite(VACUUM_ENAB_PIN, 1);
             #endif
             }
          TurnOff_Vacuum = curTime + VACUUM_ON_TIME;
          }
       }
  else { // No impuls detected
       uint32_t DeltaT = SPEEDTIMER() - LastImpTime;
       if (DeltaT > MAX_PERIOD*DURATION_FACT)
            rps = 0;
       else {
            if (rps_Period > 0 && DeltaT > rps_Period) // Ramp down the rps if it takes longer than the last period
               {
               rps = (float)RPS_FACT / DeltaT;
               }
            }
       }


  // Low pass filter for rps
  static uint32_t NextFilter_Calc = 0;
  if (curTime > NextFilter_Calc)
     {
     NextFilter_Calc = curTime + 50;
     rps_filt = 0.9*rps_filt + 0.1*rps;
     //rps_filt = rps; // DEBUG: Disable the filter
     if (rps_filt > rps_max) rps_max = rps_filt;
     }

  #if DEBUG_HALL
    if (rps < Min_rps) Min_rps = rps;  // Debug: Some times the rps measurement fails because interrupts are lost. In this time thr rps returns the half value
  #endif

  // Update the Vacuum motor timer
  static uint32_t Upd_Vacu_Time = 0;
  if (curTime - Upd_Vacu_Time > 1000)
     {
     Upd_Vacu_Time = curTime;
     if (Vacuum_State)
        {
        ++ee.Vacu_Time;
        ++ee.T_Vacu_Time;
        }
     ++ee.Total_Time;
     }

  if (curTime > TurnOff_Vacuum)
     {
     #if PWM_VACUUM_CONTROL
          analogWrite(VACUUM_ENAB_PIN, 0);
     #else
          digitalWrite(VACUUM_ENAB_PIN, 0);
     #endif
     Vacuum_State = 0;
     }

  #if RESET_TRIP_IF_STOPED
    // Restart the distance measurement if stopped for longer than RESET_TRIP_IF_STOPED [ms]
    if (Stopped_t) // Stop time has been stored
         {
         if (rps_filt >= 0.01)
            {
            if (curTime - Stopped_t > RESET_TRIP_IF_STOPED) // Started after a long break
               {
               //Serial.println(F("Reset Trip")); // Debug
               if (Enable_Distance_Measurement == 2)
                  {
                  //Serial.print("Hall_Cnt:"); Serial.print(Hall_Cnt); Serial.print(" Cnt_St:"); Serial.println(Cnt_Stopped);
                  Hall_Cnt = Hall_Cnt - Cnt_Stopped;  // We lose some counts before we detect the speed because of the filter
                  }
               rps_max = 0;
               }
            Stopped_t = 0;
            }
         }
    else { // Stopped_t == 0
         if (rps_filt < 0.01) // wagon stopped now?
            {
            Stopped_t = curTime;
            Cnt_Stopped = Hall_Cnt;
            //Serial.println(F("Stored Stopped_t")); // Debug
            Update_EEPROM();
            }
         }
  #endif
}

//--------------------------------------------------------------------------
uint8_t Read_Serial_String(char *Buffer, uint8_t BuffSize, uint16_t MaxTime)
//--------------------------------------------------------------------------
{
  *Buffer = '\0';
  uint32_t NextLED = millis();
  uint32_t Timeout = NextLED + MaxTime;
  uint8_t len = 0;
  while (millis() < Timeout)
    {
    #if LED_PIN
      if (millis() >= NextLED)
         {
         NextLED += 200;
         digitalWrite(LED_PIN, !digitalRead(LED_PIN));
         }
    #endif

    if (Serial.available() > 0)
       {
       char c = Serial.read();
       len = strlen(Buffer);
       if (c == '\n') break;
       if (c != '\r')
          {
          Buffer[len++] = c;
          Buffer[len]   = '\0';
          if (len >= BuffSize -1) break;
          }
       }
    }
  //Serial.print(F("S:'")); Serial.print(Buffer); Serial.println("'"); // Debug
  return len;
}

//--------------------
void ClearSerialBuff()
//--------------------
{
  while(Serial.available() > 0)
    {
    Serial.read();
    delay(10);
    }
}

//--------------------------------------
void Flash(uint8_t Count, uint16_t Time)
//--------------------------------------
{
  #if LED_PIN
    Count*=2;
    while (Count--)
      {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(Time);
      }
    digitalWrite(LED_PIN, 0);
  #endif
}


//----------------------------------
void Autoconfigure_Bluetooth_Modul()
//----------------------------------
// If the button on the BT module is pressed at power on the module accepts AT commands at 38400 baud.
// This is used configure the BT-Module:
// - Check if the BT-Module is in the configuartion mode
//    AT => OK
// - Configure the module:
//   - configure as slave
//   - set the password to 1234
//   - set the normal baut date to 57600 which must be equal to the flash baud rate
//   - set the name to BT_NAME (See definition above)
// - Check it the name is correct
//   Yes: Flash 5 times slow (0.5 Hz)
//   No:  Flash 20 times fast (2 Hz)
{
  if (digitalRead(BLUETOOTH_EN_PIN)) // Check if the button on the bluetooth modul is pressed
     {
     #if LED_PIN
       digitalWrite(LED_PIN, 1);
     #endif
     Serial.begin(38400);
     delay(200);
     ClearSerialBuff();
     Serial.println("AT");
     char Buff[25] = "";
     Read_Serial_String(Buff, sizeof(Buff), 4000); // normal answer time between 260 ms and 2.5 sec
     if (strcmp(Buff, "OK") == 0)
          {
          //Serial.println(F("AT+ORGL")); // Reset all  Can't be used because it resets also the Arduino
          Serial.println(F("AT+ROLE=0")); // Slave
          Serial.println(F("AT+PSWD=1234"));
          Serial.println(F("AT+UART=57600,0,0")); // Use the same baud rate as used for flashing to be able to flash the Arduino with BT
          Serial.println(F("AT+NAME=" BT_NAME));
          //Serial.println(F("AT+UART?"));          // Debug
          delay(1000); // Wait for the last "OK"
          ClearSerialBuff();
          Serial.println(F("AT+NAME?"));
          Read_Serial_String(Buff, sizeof(Buff), 5000);
          Serial.print(F("Res:'")); Serial.print(Buff); Serial.println("'");
          if (strcmp(Buff, "+NAME:" BT_NAME) == 0)
               Flash(5,1000); // O.K.
          else Flash(20,250); // Error
          Serial.println(F("AT+RESET"));           // End of the AT Configuration mode. The HC-05 LED shoul flash fast
          }                                        // Attention: The HC-05 generates a restart => A reset is

     Serial.end();
     #if LED_PIN
       digitalWrite(LED_PIN, 0);
     #endif
     }
}

//------------------------------------------------
void Set_Enable_Distance_Measurement(uint8_t Mode)
//------------------------------------------------
{
  Enable_Distance_Measurement = Mode;
  if (Mode != 2) Stopped_t = 0;
}

//--------------------------
void Clear_Vacuum_Counters()
//--------------------------
{
  ee.Vacu_Cnt  = 0;
  ee.Vacu_Time = 0;
  Update_EEPROM();
}

//-------------------------
void Set_Light(bool Enable)
//-------------------------
{
  #if LIGHT_PIN > 0
    if (Enable != ee.Light)
       {
       ee.Light = Enable;
       Update_EEPROM();
       }
    digitalWrite(LIGHT_PIN, Enable);
  #endif
}


#if DCC_ADDR >= 0
  //--------------------------------------------------------------------------------------------
  void notifyDccFunc(uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)
  //--------------------------------------------------------------------------------------------
  // DCC Commands:
  // ~~~~~~~~~~~~~
  // FN0:  On/Off   Light
  // FN1:  On/Off   Pause distance measurement
  // FN2:  On/Off   Enable vaccum motor
  // FN3:  Impuls   Clear vaccum counters
  //
  // FN4:  Impuls   Previous OLED display
  // FN5:  Impuls   Next OLED display
  // FN6:  Impuls   Set mark to identify special events (position on the track, ...)
  // FN7:  Impuls   Change Vacuum Speed
  //
  // FN8:  Impuls   Calibrate gradient and tilt angle to 0
  // FN9:
  // FN10:
  // FN11:
  //
  // FN12:
  // FN13:
  // FN14:
  // FN15:
  {
    switch(FuncGrp)
        {
        case FN_0_4: { // Function 0 - 4  (0 = Light)
                     static uint8_t OldFuncState = 0xFF;
                     #if 0 // Debug
                       Serial.print  (" FN 1-4: ");
                       Serial.print  ((FuncState & FN_BIT_01) ? "1 ": "0 ");
                       Serial.print  ((FuncState & FN_BIT_02) ? "1 ": "0 ");
                       Serial.print  ((FuncState & FN_BIT_03) ? "1 ": "0 ");
                       Serial.println((FuncState & FN_BIT_04) ? "1 ": "0 ");
                     #endif
                      #if LIGHT_PIN > 0
                           Set_Light(FuncState & FN_BIT_00);                            // FN0: Change the light
                      #endif
                     Set_Enable_Distance_Measurement((FuncState & FN_BIT_01) ? 0 : 2);  // FN1: Pause distance measurement
                     /*
                     bool New_Vacuum = FuncState & FN_BIT_02;                           // FN2: Enable Vacuum
                     static bool Act_Vacuum = 0;
                     if (Act_Vacuum != New_Vacuum)  { Change_Vacuum(New_Vacuum); Act_Vacuum = New_Vacuum; }
                     */
                     if (FuncState != OldFuncState) // Impuls functions
                        {
                        // Problem:
                        // Der Sauger kann ¸ber DCC oder ¸ber doppeltes Klopfen aktiviert werden
                        // Welcher Zustand soll verwendet werden?
                        // -> Der letzte Zustand. Das bedeutet aber, dass der DCC Zustand u.U. nicht mit
                        //    Tats‰chlichen Zustand ¸bereinstimmt.
                        //    Wenn sich die Zentrale den letzten Zustand merkt, dann wird der Zustand
                        //    im EEPROM des Saugers beim einschalten ¸berschrieben wenn man den Sauger
                        //    zuletzt per Klopfen Ein- oder Ausgeschaltet hat ohne das die Zentrale
                        //    Das mitbekommen hat.
                        //    => Das DCC Kommando wird auch nur als Taste ausgewertet.
                        if ((FuncState & FN_BIT_02) && !(OldFuncState & FN_BIT_02))     // FN3: Change Vaccuum state
                           Change_Vacuum(!ee.Vacuum);

                        if ((FuncState & FN_BIT_03) && !(OldFuncState & FN_BIT_03))     // FN3: Clear Vaccuum counters
                           Clear_Vacuum_Counters();

                        if ((FuncState & FN_BIT_04) && !(OldFuncState & FN_BIT_04))     // FN4: prev OLED Display
                           NextDisplay(-1);

                        OldFuncState = FuncState;
                        }
                     break;
                     }

        case FN_5_8: {// Function 5 - 8
                     static uint8_t OldFuncState = 0xFF;
                     #if 0 // Debug
                       Serial.print  (" FN 5-8: ");
                       Serial.print  ((FuncState & FN_BIT_05) ? "1 ": "0 ");
                       Serial.print  ((FuncState & FN_BIT_06) ? "1 ": "0 ");
                       Serial.print  ((FuncState & FN_BIT_07) ? "1 ": "0 ");
                       Serial.println((FuncState & FN_BIT_08) ? "1 ": "0 ");
                     #endif
                     if (FuncState != OldFuncState) // Impuls functions
                        {
                        if ((FuncState & FN_BIT_05) && !(OldFuncState & FN_BIT_05))     // FN5: next OLED Display
                           NextDisplay(+1);
                        if ((FuncState & FN_BIT_06) && !(OldFuncState & FN_BIT_06))     // FN6: next OLED Display
                           ZMark = 1;
                     #if PWM_VACUUM_CONTROL
                        if ((FuncState & FN_BIT_07) && !(OldFuncState & FN_BIT_07))     // FN7: Change Vacuum Speed
                           {
                           ee.VacuumSpeed += 64; // 63, 127, 192, 255
                           if (ee.VacuumSpeed < 63) ee.VacuumSpeed = 63;
                           Update_EEPROM();
                           Change_Vacuum(1);
                           }
                     #endif
                        if ((FuncState & FN_BIT_08) && !(OldFuncState & FN_BIT_08))     // FN8: Calibrate gradient and tilt angle to 0
                           MeasureFilteredAngleOffset();
                        OldFuncState = FuncState;
                        }
                     break;
                     }
        case FN_9_12:{// Function 9 - 12
                     static uint8_t OldFuncState = 0xFF;
                     #if 0 // Debug
                       Serial.print  (" FN 9-12: ");
                       Serial.print  ((FuncState & FN_BIT_09)  ? "1 ": "0 ");
                       Serial.print  ((FuncState & FN_BIT_010) ? "1 ": "0 ");
                       Serial.print  ((FuncState & FN_BIT_011) ? "1 ": "0 ");
                       Serial.println((FuncState & FN_BIT_012) ? "1 ": "0 ");
                     #endif
                     if (FuncState != OldFuncState) // Impuls functions
                        {
                     #if ENABLE_DISPLAY_ROTATION
                        if ((FuncState & FN_BIT_09) && !(OldFuncState & FN_BIT_09))     // FN8: Flip the display
                           {
                           ee.FlipMode = !ee.FlipMode;
                           u8x8.setFlipMode(ee.FlipMode);
                           UpdateOLED = 0; // Update now
                           Update_EEPROM();
                           }
                     #endif
                        OldFuncState = FuncState;
                        }
                     break;
                     }
        default: ;// Nothing (Prevent compiler warning)
        }
  }

  // It's not possible to disable the writing to the EEPROM in the NmraDcc library
  // except by defining the following functions which simulate the EEPROM
  //------------------------------------------------
  uint8_t notifyCVWrite( uint16_t CV, uint8_t Value)
  //------------------------------------------------
  {
    //{ char Buff[20]; sprintf(Buff, "notifyCVWrite %i %i", CV, Value); Serial.println(Buff);}
    #if ENAB_CV_ADR_CHANGE
      // The address could be changed, but there is no feedback to the control station
      // because the arduino has no output to generate the ack signal.
      // => An error will be shown, but the CV is written
      //
      // Change the address:
      // ~~~~~~~~~~~~~~~~~~~
      // Short adress: write the adress (1..127) to CV1
      //
      // Long address:
      // Write to CV17 and CV18
      // CV17 = 192 + (Addr / 256)
      // CV18 = Addr % 256
      // CV29 is set automatically and dont have to be written (but it could)
      //
      // Example: Addr 642 => CV17 = 194, CV18 = 130
      //   Calculation see: https://bluethners.de/DCCProjekt/Zentrale/Lokadresse.html

      uint8_t AddrChg = 0;
      if (CV == 1)  { AddrChg = 1; ee.DCC_Adress = Value; }
      if (CV == 17) { AddrChg = 1; ee.DCC_Adress = (ee.DCC_Adress & 0x00FF) | (Value - 192) * 256; }
      if (CV == 18) { AddrChg = 1; ee.DCC_Adress = (ee.DCC_Adress & 0xFF00) | Value;               }
      if (AddrChg)
         {
         Serial.print(F("Changed DCC Address to:")); Serial.println(ee.DCC_Adress);
         Update_EEPROM();
         }

    #endif
    return Value;
  }

  //--------------------------------
  uint8_t notifyCVRead( uint16_t CV)
  //--------------------------------
  {
    uint8_t ret = 255;
    switch (CV)
        {
        case  7: ret = 10;                           break; // Version ID returned in CV 7.
        case  8: ret = MAN_ID_DIY;                   break; // Manufacturer ID returned Commonly
      #if ENAB_CV_ADR_CHANGE
        case  1: ret = ee.DCC_Adress % 128;          break; // Primary Address
        case 17: ret = 192 + (ee.DCC_Adress >> 8);   break;
        case 18: ret = ee.DCC_Adress % 256;          break;
        case 29: ret = 2 + (ee.DCC_Adress>127?32:0); break; // Configuration Data #1
      #else // Address can't be changed with CV's. It's defined above with #define DCC_ADDR ...
        case  1: ret = DCC_ADDR % 128;               break; // Primary Address
        case 17: ret = 192 + (DCC_ADDR >> 8);        break;
        case 18: ret = DCC_ADDR % 256;               break;
        case 29: ret = 2 + (DCC_ADDR>127?32:0);      break; // Configuration Data #1
      #endif
        }
    //{ char Buff[20]; sprintf(Buff, "notifyCVRead %i:%i", CV, ret); Serial.println(Buff);}
    return ret;
  }

//   Configuration Variable 1    Primary Address
//   ~~~~~~~~~~~~~~~~~~~~~~~~
//    Bits 0-6 contain an address with a value between 1 and 127.
//    Bit seven must have a value of "0".   If the value 45 of Configuration Variable #1
//    is "00000000" then the decoder will go out of NMRA digital mode and convert to
//    the alternate power source as defined by Configuration Variable #12.
//    This setting will not affect the Digital Decoder's ability to respond to service mode packets (see S 9.2.3).
//    The default value for this Configuration Variable is 3, if the decoder is not installed in a
//    locomotive or other unit when shipped from the manufacturer.
//
//   Configuration Variable 7    Manufacturer Version Number
//   ~~~~~~~~~~~~~~~~~~~~~~~~
//    This is reserved for the manufacturer to store information regarding the version of the decoder.
//
//   Configuration Variable 8    Manufacturer ID
//   ~~~~~~~~~~~~~~~~~~~~~~~~
//    CV8 shall contain the NMRA assigned id number of the manufacturer of this decoder. The currently assigned manufacturer ID codes are listed in Appendix A of this Standard. The use of a value not assigned by the NMRA shall immediately cause the decoder to not be in conformance to this Standard.  The CV shall be implemented as a read-only value, which cannot be modified.
//
//   Configuration Variable 29   Configurations Supported
//   ~~~~~~~~~~~~~~~~~~~~~~~~~
//    Bit 0 = Locomotive Direction: "0" = normal, "1" = reversed.
//            This bit controls the locomotive's forward and 215 backward direction in digital mode only.
//            Directional sensitive functions, such as headlights (FL and FR), will also be reversed so that
//            they line up with the locomotiveís new forward direction. See S-9.1.1 for more information.
//    Bit 1 = FL location:
//            "0" = bit 4 in Speed and Direction instructions control FL,
//            "1" = bit 4 in function group one instruction controls FL. See S-9.2.1 for more information.
//    Bit 2 = Power Source Conversion:  "0" = NMRA Digital Only, "1" = Power Source Conversion Enabled,
//            See CV#12 for more information,
//    Bit 3 = Bi-Directional Communications:
//            "0" = Bi-Directional Communications disabled,
//            "1" = Bi-Directional Communications enabled.  See S-9.3.2 for more information.
//    Bit 4 = Speed Table:
//            "0" = speed table set by configuration variables #2,#5, and #6,
//            "1" = Speed Table set by 225 configuration variables #66-#95
//    Bit 5 = "0" = one byte addressing,
//            "1" = two byte addressing (also known as extended addressing), See S 9.2.1 for more information.
//    Bit 6 = Reserved for future use.
//    Bit 7 = Accessory Decoder:
//            "0" = Multifunction Decoder,
//            "1" = Accessory Decoder (see CV #541 for a 230 description of assignments for bits 0-6)
//    *Note If the decoder does not support a feature contained in this table, it shall not allow the
//     corresponding bit to be set improperly (i.e. the bit should always contain its default value).
#endif // DCC_ADDR


//----------
void setup()
//----------
{
#if RESET_PULLUP
  digitalWrite(RESET_PULLUP, 1);
  pinMode(RESET_PULLUP, OUTPUT);
#endif
  pinMode(HALLPIN,               INPUT_PULLUP);
#if LED_PIN
  pinMode(LED_PIN,               OUTPUT);
#endif
  pinMode(BLUETOOTH_EN_PIN,      INPUT);
  pinMode(BLUETOOTH_STATE_PIN,   INPUT);
  pinMode(BLUETOOTH_DISABLE_PIN, OUTPUT);
  pinMode(VACUUM_ENAB_PIN,       OUTPUT);
  #if PWM_VACUUM_CONTROL
    setPwmFrequency(VACUUM_ENAB_PIN, 1); // ~32 kHz
  #endif
#if ZERO_MARK_HALL_PIN
  pinMode(ZERO_MARK_HALL_PIN,    INPUT_PULLUP);
#endif

  attachInterrupt(digitalPinToInterrupt(HALLPIN), Hall_Int_Proc, FALLING);

  #if OLED_DISP > 0
    u8x8.begin();
    #if OLED_CONTRAST >= 0
      u8x8.setContrast(OLED_CONTRAST);
    #endif
    u8x8.setFont(FONT_NAME);

    #if OLED_Lines >= 4
      u8x8.drawString(1, 2, ":)*+,/");    // Start screen using special characters
      u8x8.drawString(3, 5, "(" VERSTR);
    #else
      u8x8.drawString(1, 0, ":)*+,/");
      u8x8.drawString(3, 2, "(" VERSTR);
    #endif
  #endif

  Autoconfigure_Bluetooth_Modul(); // Uses 3% FLASH

  Serial.begin(57600); // Must be the same baud rate which is used for flashing the arduino
  Serial.println(F("BT Messw." VERSTR));

  #if GRADIENT_MEASUREMENT
    Wire.begin();
    if (!myAcc.init()) Serial.println(F("ADXL not con."));
    myAcc.setDataRate(ADXL345_DATA_RATE_50);
    #if ENABLE_TAP_DETECTION
      //attachInterrupt(digitalPinToInterrupt(int1Pin), tapISR, RISING);
      // The following four parameters have to be set for tap application (single and double):
      //  1. Axes, that are considered:
      //      ADXL345_000  -  no axis (which makes no sense)
      //      ADXL345_00Z  -  z
      //      ADXL345_0Y0  -  y
      //      ADXL345_0YZ  -  y,z
      //      ADXL345_X00  -  x
      //      ADXL345_X0Z  -  x,z
      //      ADXL345_XY0  -  x,y
      //      ADXL345_XYZ  -  all axes
      //  2. Threshold in g
      //      It is recommended to not choose the value to low. 3g is a good starting point.
      //  3. Duration in milliseconds (max 159 ms):
      //      maximum time that the acceleration must be over g threshold to be regarded as a single tap. If
      //      the acceleration drops below the g threshold before the duration is exceeded an interrupt will be
      //      triggered. If also double tap is active an interrupt will only be triggered after the double tap
      //      conditions have been checked. Duration should be greater than 10.
      //  4. Latency time in milliseconds (maximum: 318 ms): minimum time before the next tap can be detected.
      //      Starts at the end of duration or when the interrupt was triggered. Should be greater than 20 ms.
      myAcc.setGeneralTapParameters(ADXL345_0Y0, TAP_THRESHOLD, 30, 100.0);

      // For double tap detection additional parameters have to be set:
      //  1. Suppress bit: if the bit is set, a spike over the threshold during the latency time will invalidate
      //      the double tap. You find graphs that explain this better than words in the data sheet. If the bit is
      //      not set, the tap during latency time will be ignored. Another tap in the time window (see 2) will lead
      //      to double tap detection.
      //  2. Time window in milliseconds: starts after latency time. The second tap must occur during the time window
      //      period, otherwise it is a single tap (if single tap is active). Maximum window is 318 ms.
      myAcc.setAdditionalDoubleTapParameters(false, DOUBLE_TAB_TIME_WINDOW);

      //  You can choose the following interrupts:
      //      Variable name:             Triggered, if:
      //     ADXL345_OVERRUN      -   new data replaces unread data
      //     ADXL345_WATERMARK    -   the number of samples in FIFO equals the number defined in FIFO_CTL
      //     ADXL345_FREEFALL     -   acceleration values of all axes are below the threshold defined in THRESH_FF
      //     ADXL345_INACTIVITY   -   acc. value of all included axes are < THRESH_INACT for period > TIME_INACT
      //     ADXL345_ACTIVITY     -   acc. value of included axes are > THRESH_ACT
      //     ADXL345_DOUBLE_TAP   -   double tap detected on one incl. axis and various defined conditions are met
      //     ADXL345_SINGLE_TAP   -   single tap detected on one incl. axis and various defined conditions are met
      //     ADXL345_DATA_READY   -   new data available
      //
      //  Assign the interrupts to INT1 (INT_PIN_1) or INT2 (INT_PIN_2). Data ready, watermark and overrun are
      //  always enabled. You can only change the assignment of these which is INT1 by default.
      //
      //  You can delete interrupts with deleteInterrupt(type);
      myAcc.setInterrupt(ADXL345_SINGLE_TAP, INT_PIN_1);
      myAcc.setInterrupt(ADXL345_DOUBLE_TAP, INT_PIN_1);
      myAcc.setRange(ADXL345_RANGE_8G);
    #else
      myAcc.setRange(ADXL345_RANGE_2G);
    #endif // ENABLE_TAP_DETECTION
  #endif // GRADIENT_MEASUREMENT

  // Wozu diese Ausgabe ?
  /*
  char buffervar[15];
  sprintf(buffervar, "#%05i-%05i~", 0, 0);
  Serial.print(buffervar);
  delay(1000);
  Serial.print(buffervar);
  */

  analogReference(INTERNAL); // 1.1V

  ReadEEPROM();

  #if OLED_DISP > 0
    #if ENABLE_DISPLAY_ROTATION
      u8x8.setFlipMode(ee.FlipMode);
    #endif
  #endif

  #if DCC_ADDR >= 0
    Serial.print(F("DCC_Addr_"));
    #if ENAB_CV_ADR_CHANGE
      Serial.print(ee.DCC_Adress);  // No underscore between txt and number to prevent that the serial plotter changes the y-scale
    #else
      Serial.println(DCC_ADDR);
    #endif
    Dcc.pin(0, 2, 0); // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up
    Dcc.init( MAN_ID_DIY, 10, FLAGS_MY_ADDRESS_ONLY, 0 );
  #endif // DCC_ADDR

  Serial.print(F(" ScaleFact_")); Serial.print(ee.ScaleFact); // To be able to calculate the physical values in the aplication
  Serial.print(F(" Perimeter_")); Serial.println(ee.Perimeter,8);

  if (!ee.OffsetCorrected)
     MeasureFilteredAngleOffset();
}

//-----------------
void Update_EEPROM()
//-----------------
// Save all changed values to the EEPROM
{
  uint8_t *p = (uint8_t*)&ee;
  for (uint16_t Addr = EE_START_ADDR; Addr < sizeof(EE_Data_T)+EE_START_ADDR; ++Addr, ++p)
      {
      EEPROM.update(Addr, *p);
      }
}

/*
 Berechnung:
 ~~~~~~~~~~
 SpeedScale:                                      1km    60 Min
   Speed = Umfang * Anz/Zeit * 87   [mm / Min]    -----  ----
                                                  1e6mm   1 h
   D = 11mm
   Umfang = D * Pi / 2 = 17.279mm    "/ 2" Weil zwei Magnete verwendet werden
   Speed = SpeedScale * rps_filt
   rps_filt = upm * 8
   SpeedScale = Umfang * 87 * 60 / 1e6 / 8 = 0.0112744

  Perimeter:
   Dist = Umfang * Count / 1000 =  0.01728 [m]
  Eine Viadukt Runde:
   WinTrack:  4.7m
   Messwagen: 4.65m

   SpeedScale = Perimeter * 87 * 60 / 1000

*/


//---------------
void ReadEEPROM()
//---------------
{
  memset(&ee, 0, sizeof(ee));
  strcpy(ee.Ver, EE_VERSION_STR);
  ee.Ver_Nr          = EE_VER_NR;
  ee.DCC_Addr_Def    = DCC_ADDR;
  // Default values in case the EEPROM is initialized
  ee.Vacuum          = 1;
  ee.ScaleFact       = DEF_SCALEFACT;          // H0: 1/87  1/22.5 Industriebahn
  ee.Perimeter       = DEF_PERIMETER;
  ee.DCC_Adress      = DCC_ADDR;
  ee.VacuumSpeed     = 127;

  uint8_t *p = (uint8_t*)&ee;
  uint8_t b;
  uint16_t Addr;
  for (Addr = EE_START_ADDR; Addr < sizeof(EE_Data_T)+EE_START_ADDR; ++Addr, ++p)
    {
    b = EEPROM.read(Addr);
    if (Addr < EE_CHECK_SIZE)
         {
         //char buff[30]; sprintf(buff, "%i %c=%02X %02X\n", Addr, b<' ' ? '?':b, b, *p); Serial.print(buff);// Debug
         if (b != *p)
            {
            //Serial.println(F("Writing EEPROM defaults")); // Debug
            Update_EEPROM();
            break;
            }
         }
    else *p = b;
    }
  SpeedScale = ee.Perimeter * ee.ScaleFact * 60 / 1000;
  #if LIGHT_PIN > 0
    digitalWrite(LIGHT_PIN, ee.Light);
  #endif
}

//-------------------------------
void MeasureFilteredAngleOffset()
//-------------------------------
{
  #if GRADIENT_MEASUREMENT
    xyzFloat angle = myAcc.getAngles();
    ee.AngleOffsetX = angle.x;
    ee.AngleOffsetZ = angle.z;
    //Serial.print(F("Callibrating angle offset..."));
    #if OLED_DISP
      u8x8.clearDisplay();
      u8x8.drawString(2,0, S_GRD " 0...");
      u8x8.drawString(2,2, S_TLD " 0...");
      UpdateOLED = millis() + 4000;
    #endif

    //Serial.print(F("AngleOffset:")); Serial.println(ee.AngleOffsetX);
    uint32_t EndTime = millis() + 2000;
    float Filt = 0.97;
    while (millis() < EndTime)
      {
      angle = myAcc.getAngles();
      ee.AngleOffsetX = Filt * ee.AngleOffsetX + (1-Filt) * angle.x;
      ee.AngleOffsetZ = Filt * ee.AngleOffsetZ + (1-Filt) * angle.z;
      //Serial.print(F("AngleOffset: ")); Serial.println(ee.AngleOffsetX);
      delay(20);
      }
    //Serial.print(F("AngleOffset:")); Serial.print(ee.AngleOffsetX);
    ee.OffsetCorrected = 1;
    Update_EEPROM();
  #endif
}

//--------------------------------
void Change_Vacuum(uint8_t Enable)
//--------------------------------
// Turn on/off the vacuum motor.
// If the vacuum motor is stopped because the train is not moving
// a status change is indicated by turning on the motor for a
// short time.
//  1 second: if the vacuum motor is enabled
//  100 ms:   if the vacuum motor is disabled
{
  //Serial.print(F("Change_Vacuum ")); Serial.println(Enable);
  uint8_t Speed = Enable?ee.VacuumSpeed:123; // Medium Speed to indicate that the motor is turned off because the short impulse is not noticeable if slow speed is used.
  if (Vacuum_State == 0) // Not active at the moment => turn on the motor to indicate that the command was received
       {
       Vacuum_State = 1;
       uint32_t ms = millis();
       if (Enable)
            TurnOff_Vacuum = ms + 1000;
       else TurnOff_Vacuum = ms + 100;
       }
  else { // Vacuum motor is running
       if (Enable == 0)
          {
          Vacuum_State = 0;
          Speed = 0;
          }
       }

  #if VACU_SUPERCAP_VOLT_CONTR
    if (!Vacuum_Voltage_Ok) Speed = 0;
  #endif

  #if PWM_VACUUM_CONTROL
       analogWrite(VACUUM_ENAB_PIN, Speed);
  #else
       digitalWrite(VACUUM_ENAB_PIN, Speed);
  #endif
  ee.Vacuum = Enable;
  Update_EEPROM();
}

//--------------------------
void NextDisplay(int8_t Dir)
//--------------------------
{
  #if OLED_DISP > 0
    if (Dir > 0)
         {
         ee.ActOLEDDisp += OLED_Lines;
         if (ee.ActOLEDDisp >= OLED_TOTAL_LINES)
         ee.ActOLEDDisp = 0;
         }
    else {
         if (ee.ActOLEDDisp == 0)
              ee.ActOLEDDisp = OLED_TOTAL_LINES - OLED_Lines;
         else if (ee.ActOLEDDisp - OLED_Lines >= 0)
              ee.ActOLEDDisp -= OLED_Lines;
         else ee.ActOLEDDisp = 0;
         }
    UpdateOLED = 0; // Update now
    Update_EEPROM();
  #endif
}



//-----------------------------------------
void SPrint8_Dist(uint32_t Cnt, char *Buff)
//-----------------------------------------
{
  float Dist = (float)ee.Perimeter*Cnt;
  if (Dist < 10000)                                              // " 0.000m"  - "9999.9m"
       { dtostrf(Dist,      6,3, Buff); strcpy(Buff+6, S_m);}
  else if (Dist  < 1000000)                                    // " 10000m" - "999999m"
       { dtostrf(Dist,      6,0, Buff); strcpy(Buff+6, S_m);}
  else { dtostrf(Dist/1000, 6,0, Buff); strcpy(Buff+6, S_km);} // " 1000km" - "99999km" (uint32_t cnt => @ d= 5.5mm 74212 km)
}

//------------------------------------------
void SPrint8_Time(uint32_t Time, char *Buff)
//------------------------------------------
//  1234567
// "999m59s"
// "999h59m"
{
  uint32_t h = Time / 3600;
  uint16_t m = (Time - h*60) / 60;
  uint8_t  s = Time % 60;
  if (m > 999)
       sprintf(Buff, "%3li" S_h "%02i" S_m, h, m/60);
  else sprintf(Buff, "%3i"  S_m "%02i" S_s, m, s);
}

//----------------
void Update_OLED()
//----------------
// In front of each line a special character is shown. It is stored in
//   u8x8_font_px437wyse700a_2x2_rx_Add_Sym.h
//
// Displays:
// ~~~~~~~~~
// 12345678
//     450km/h Speed
//  35.730m    Distance
//    5.23%    Gradient = Steigung
//    1.25∞    Tilt = Neigung
//   1.272m    Vacuum distace
//    0m14s    Vacuum time
//  123456km   Total Distance
//  334h23m    Total Time
//    4322km   Total Vacuum distace
//   32h32m    Total Vacuum time
//    12.1V    Voltage Rail
//     4.5V    Voltage Ext.
//     4.9V    Voltage Arduino
//
{
#if OLED_DISP > 0
   uint32_t t = millis();
   if (t <= UpdateOLED) return ;
   UpdateOLED = t + 1000;
   uint8_t dNr = ee.ActOLEDDisp;
   for (uint8_t i = 0; i < OLED_Lines; ++i,++dNr)
     {
     char Line[15]; // Additional characters to be able to shift the display 12345.678m => 12345.67m
     dNr %= OLED_TOTAL_LINES;
     *Line = First_ICON + dNr; // special icon stored in the font
     switch (dNr)
       { // Don't change the sequece of the displays because the icon characters in the font   // 12345678
       case 0:  dtostrf(SpeedScale*60*rps_filt, 6,0, Line+1); strcpy(Line+7, S_KMH);    break; //     450km/h  (km/h = one character)
       case 1:  SPrint8_Dist(Hall_Cnt,               Line+1);                           break; //  12.345m
       case 2:  dtostrf(Filt_Gradient,          6,2, Line+1); strcpy(Line+7, S_PCT);    break; //  -10.23%   // case the number is longer then expected
       case 3:  dtostrf(Filt_Tilt,              6,2, Line+1); strcpy(Line+7, S_DEG);    break; //  -5.3∞
       case 4:  SPrint8_Dist(ee.Vacu_Cnt,            Line+1);                           break; //  12.345m
       case 5:  SPrint8_Time(ee.Vacu_Time,           Line+1);                           break; //  123m23s
       case 6:  SPrint8_Dist(ee.Total_Cnt,           Line+1);                           break; //  12.345m
       case 7:  SPrint8_Time(ee.Total_Time,          Line+1);                           break; //  123m23s
       case 8:  SPrint8_Dist(ee.T_Vacu_Cnt,          Line+1);                           break; //  12.345m
       case 9:  SPrint8_Time(ee.T_Vacu_Time,         Line+1);                           break; //  123m23s
       case 10: dtostrf((float)U0/1000,         6,1, Line+1); strcpy(Line+7, S_V);      break; //   12.1V
       case 11: dtostrf((float)U1/1000,         6,1, Line+1); strcpy(Line+7, S_V);      break; //    4.5V
       case 12: dtostrf((float)U6/1000,         6,1, Line+1); strcpy(Line+7, S_V);      break; //    4.5V
       case 13: sprintf(Line+1, "%6i ", ee.DCC_Adress);                                 break; // DCC10239
       case 14: dtostrf(SpeedScale*60*rps_max, 6,0,  Line+1); strcpy(Line+7, S_KMH);    break; //     450km/h  (km/h = one character)
       // Attention: If case statements are changed adapt also OLED_TOTAL_LINES
       }
     u8x8.drawString(0, i*2, Line);
     Check_Hall();
     }
#endif
}

//--------------------------------------------
void Read_Number(char* Buff, uint8_t BuffSize)
//--------------------------------------------
{
  uint32_t End_Time = millis() + 10;
  char *p = Buff, *e = Buff+BuffSize-1;
  while (millis() < End_Time && p < e)
    {
    if (Serial.available() > 0)
       {
       char c = Serial.read();
       if (c == '\n')
           {
           *p = '\0';
           Serial.print(F("Read_Nr:\"")); Serial.print(Buff); Serial.println('"');
           return ;
           }
       else if ((c >= '0' && c <= '9') || c == '.')
               *(p++) = c;
       }
    }
  *Buff = '\0';
}

//-----------------------
void Read_Float(float &f)
//-----------------------
{
  char Buff[21];
  Read_Number(Buff, sizeof(Buff));
  if (*Buff)
     {
     f = atof(Buff);
     Update_EEPROM();
     ReadEEPROM();
     }
}


//---------
void loop()
//---------
{
  #if DEBUG_MAINLOOP_CHECK
    static uint32_t Max_Loop_Time      = 0;
    static uint16_t Loop_cnt           = 0;
    static uint32_t Avg_Loop_Start;
    static uint32_t Max_Loop_Time_Disp = 0;
    static uint32_t Avg_Loop_Time_Disp = 0;
    uint32_t startLoop = micros();
  #endif

  curTime = millis();

  #if DCC_ADDR >= 0
    Dcc.process();
  #endif

  Check_Hall();  // The main loop time should be short to capture all hall sensor events


#if GRADIENT_MEASUREMENT
  #if ENABLE_TAP_DETECTION
    // Tap detection
    static uint32_t Tab_Delay = 0;
    if (digitalRead(TAP_ARDU_PIN)) tap = true;
    if (tap == true)
       {
       if (Tab_Delay == 0)
            Tab_Delay = curTime + DOUBLE_TAB_TIME_WINDOW;
       else if (curTime > Tab_Delay)
               {
               Tab_Delay = 0;
               byte intSource = myAcc.readAndClearInterrupts();
               if (rps_filt < 0.01 && curTime > TurnOff_Vacuum + 1000) // Only active if the wagon is not moving
                  {
                  if (myAcc.checkInterrupt(intSource, ADXL345_DOUBLE_TAP))
                      {
                      //Serial.println(F("Double tap")); // Debug
                      Change_Vacuum(!ee.Vacuum);
                      }
                  else if (myAcc.checkInterrupt(intSource, ADXL345_SINGLE_TAP))
                      {
                      //Serial.println(F("Single tap")); // Debug
                      NextDisplay(1);
                      }
                  }
               tap = false;
               }
       }
  #endif // ENABLE_TAP_DETECTION

  #if ZERO_MARK_HALL_PIN > 0
      if (digitalRead(ZERO_MARK_HALL_PIN) == 0) ZMark = 7; // 7 to be more visible in the serial plotter
  #endif


  // Gradient measurement
  static uint32_t NextGradientCheck = 0;
  if (curTime > NextGradientCheck)
       {
       xyzFloat corrAngles= myAcc.getAngles();
       corrAngles.x -= ee.AngleOffsetX;
       corrAngles.z -= ee.AngleOffsetZ;
       float Gradient = 100*tan(2*PI/360*corrAngles.z);  // Gradient checked with 10% => Disp. ~10.7%
       float Filt = 0.97;
       if (NextGradientCheck != 0)
            { Filt_Gradient = Filt * Filt_Gradient + (1-Filt) * Gradient;  Filt_Tilt = Filt * Filt_Tilt + (1-Filt) * corrAngles.x; }
       else { Filt_Gradient = Gradient;                                    Filt_Tilt = corrAngles.x; }
       NextGradientCheck = curTime + 20;
       }
  else // The gradient measurements are quite slow => Don't do anything else in this main loop call
#endif // GRADIENT_MEASUREMENT
       {
       if (curTime - showTime > SHOW_INTERVAL)
            {
            showTime = curTime;
            char buffervar[25];
            // To show the values with the serial plotter of the Arduio IDE there should be
            // - a colon ':' between variable name and value
            // - no space characters in between

            #if 1 // Old format for the smart phone app "Messwagen" from Matthias. Attention: Changing the diameter in current app doesn't work. The diameter is set fix to 11mm ;-(
              sprintf(buffervar, "#%05li+%05li~ ", Hall_Cnt, (uint32_t)(rps_filt*60));
              Serial.print(buffervar);
            #endif

            // Speed in rps
            sprintf(buffervar, "Cnt%-5li rps:", Hall_Cnt); Serial.print(buffervar); // rps = Hz
            dtostrf(rps_filt, 1, 2, buffervar);            Serial.print(buffervar); // Using rps to be able to plot it with the seriel plotter in the same range as the other values
              Serial.print(F(" ZMark:"));  Serial.print(ZMark);
              ZMark = 0;

            // Voltage measurement
            U0 = (5015L * analogRead(RAIL_VOLT_PIN)) / 256; // mV
            U1 = (1447L * analogRead(Ext_VOLT_PIN))  / 256; // mV
            U6 = (1447L * analogRead(INT_VOLT_PIN))  / 256; // mV

            #if SERIAL_PRINT_VALUES
              Serial.print(F(" U0:")); Serial.print((float)U0/1000);
              Serial.print(F(" U1:")); Serial.print((float)U1/1000);
              Serial.print(F(" U6:")); Serial.print((float)U6/1000);
              #if GRADIENT_MEASUREMENT
                Serial.print(F(" Steig:"));  dtostrf(Filt_Gradient, 1,2, buffervar); Serial.print(buffervar);
                Serial.print(F(" Neig:"));   dtostrf(Filt_Tilt,     1,1, buffervar); Serial.print(buffervar);
              #endif

              Serial.print(F(" Speed_")); dtostrf(SpeedScale*60*rps_filt,1,0, buffervar);Serial.print(buffervar);
              #if 1 // Counters stored in the EEPROM
                Serial.print(F(" VCnt_" ));  Serial.print(ee.Vacu_Cnt);
                Serial.print(F(" VTime_"));  Serial.print(ee.Vacu_Time);
                Serial.print(F(" TVCnt_"));  Serial.print(ee.T_Vacu_Cnt);
                Serial.print(F(" TVTime_")); Serial.print(ee.T_Vacu_Time);
                Serial.print(F(" T_Cnt_"));  Serial.print(ee.Total_Cnt);
                Serial.print(F(" T_Time_")); Serial.print(ee.Total_Time);
              #endif

              #if MIN_VOLTAGE_LINE > 0
                Serial.print(F(" ")); Serial.print(MIN_VOLTAGE_LINE);
              #endif

              #if 0  // Debug bluetooth state
                Serial.print(F(" BT"));    Serial.print(BT_State);           // No space to be shown in the top line of the Arduino serial plotter
                Serial.print(F("_"));      Serial.print(digitalRead(BLUETOOTH_STATE_PIN));
              #endif

              #if DEBUG_MAINLOOP_CHECK
              //Serial.print(F(" LMax ")); Serial.print((float)Max_Loop_Time_Disp/1000); Serial.print(F(" ms LAvg ")); Serial.print((float)Avg_Loop_Time_Disp/1000); Serial.print(F(" us")); // Show in the serial plotter as graph
                Serial.print(F(" LMax_")); Serial.print((float)Max_Loop_Time_Disp/1000); Serial.print(F( "ms LAvg_")); Serial.print((float)Avg_Loop_Time_Disp/1000); Serial.print(F("us"));  // Don't show as graph
                Max_Loop_Time_Disp = 0;
              #endif

              #if DEBUG_HALL
                Serial.print(F(" MultiHall_")); Serial.print(Multi_Hall_Pulses);
                Serial.print(F(" Min_ups "));   Serial.print(Min_rps); Min_rps = 9999L; // Debug
              #endif

              Serial.print('\n');
            #endif // SERIAL_PRINT_VALUES

            #if ENABL_BT_LOW_VOLT_DETECTION
              // Disable and restart the bluetooth module if the supply voltage is to low
              switch (BT_State)
                {
                case BT_NORMAL:   if (curTime > Check_Supply_Volt_t && U6 < BT_VOLT_LOW) // 4.35V
                                     {
                                     BT_State = BT_OFF;
                                     digitalWrite(BLUETOOTH_DISABLE_PIN, 1);
                                     Check_Supply_Volt_t = curTime + 2000;    // Disable the Bluetooth for at least 2 seconds
                                     digitalWrite(BLUETOOTH_RESETLOCK_PIN,1); // Switch to +5V to disable the BT Reset because otherwise the
                                     pinMode(BLUETOOTH_RESETLOCK_PIN, OUTPUT);// arduino is resetted at power on.
                                     }
                                  break;
                case BT_OFF:      if (curTime > Check_Supply_Volt_t && U6 > BT_VOLT_NORM)
                                     {
                                     BT_State = BT_STARTING;
                                     digitalWrite(BLUETOOTH_DISABLE_PIN, 0); // Enable the BT Power
                                     Check_Supply_Volt_t = curTime + 20000;  // Give the external program time to reconnect
                                     }
                                  break;
                case BT_STARTING: if (U6 > BT_VOLT_NORM)
                                     {
                                     if (curTime > Check_Supply_Volt_t)
                                        {
                                        BT_State = BT_NORMAL;
                                        pinMode(BLUETOOTH_RESETLOCK_PIN, INPUT); // Enable the normal reset pin function again
                                        }
                                     }
                                  else if (U6 < BT_VOLT_LOW) // The voltage dropped again below the critical value
                                     {
                                     BT_State = BT_OFF;
                                     digitalWrite(BLUETOOTH_DISABLE_PIN, 1);
                                     Check_Supply_Volt_t = curTime + 2000;   // Disable the Bluetooth for et least 2 seconds
                                     pinMode(BLUETOOTH_RESETLOCK_PIN, OUTPUT); // Switch to +5V to disable the BT Reset because otherwise the
                                     }
                                  break;
                }
            #endif // ENABL_BT_LOW_VOLT_DETECTION

            // Check if the data have to be saved to the EEPROM
            // The data are only saved if the voltage doesn't drop to fast (SuperCap available)
            static uint8_t EE_SAVE_State = 0;
            static uint32_t Low_Volt_Det_Time = 0;
            if (U6 > BT_VOLT_NORM)
                 EE_SAVE_State = 0;
            else {
                 switch (EE_SAVE_State)
                     {
                     case 0: if (U6 < BT_VOLT_LOW)
                                {
                                Low_Volt_Det_Time = curTime;
                                EE_SAVE_State = 1;
                                }
                             break;
                     case 1: if (curTime - Low_Volt_Det_Time > 100)
                                { // There must be a SuperCap => We could write to the EEPROM
                                //Serial.println(F("Saving counters"));
                                Update_EEPROM();
                                EE_SAVE_State = 2;
                                }
                     }
                 }
            }
       else Update_OLED(); // takes very long ;-(   (4 lines 96": 110 ms, 2 lines 91": 60ms)

       #if VACU_SUPERCAP_VOLT_CONTR
           if (U6 > VACU_ENAB_VOLT)
                {
                if (Vacuum_Voltage_Ok == 0)
                   {
                   Vacuum_Voltage_Ok = 1;
                   pinMode(VACUUM_ENAB_PIN, OUTPUT);
                   }
                }
           else if (U6 < VACU_DISAB_VOLT)
                   {
                   if (Vacuum_Voltage_Ok)
                      {
                      Vacuum_Voltage_Ok = 0;
                      pinMode(VACUUM_ENAB_PIN, INPUT);
                      }
                   }
       #endif // VACU_SUPERCAP_VOLT_CONTR

       // Heartbeat
       #if LED_PIN
         static uint32_t NextT = 0;
         if (curTime > NextT)
            {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            if (ee.Vacuum)
                 NextT = curTime + (Enable_Distance_Measurement ? 500 : 1500);
            else {
                 if (digitalRead(LED_PIN))    // Short flash if Vacuum is disabled
                      NextT = curTime + 100;
                 else NextT = curTime + (Enable_Distance_Measurement?900:2900);
                 }
            }
       #endif

       // Process incomming commands
       if (Serial.available() > 0)
         {
         char inbyte = Serial.read();
         switch (inbyte)
             {
             case '0': Set_Enable_Distance_Measurement(0); break;
             case '1': Set_Enable_Distance_Measurement(1); break;
             case '2': Set_Enable_Distance_Measurement(2); break; // Restart if stopped longer then RESET_TRIP_IF_STOPED [ms]
             case 'v': Change_Vacuum(0);                   break;
             case 'V': Change_Vacuum(1);                   break;
             case 'z': MeasureFilteredAngleOffset();       break;
             case '+': NextDisplay( 1);                    break;
             case '-': NextDisplay(-1);                    break;
             case 'c': Clear_Vacuum_Counters();            break;
             case 'U': Read_Float(ee.Perimeter);           break;
             case 'S': Read_Float(ee.ScaleFact);           break;
           #if LIGHT_PIN > 0
             case 'l': Set_Light(0);                       break;
             case 'L': Set_Light(1);                       break;
           #endif
             }
         }
       }

  #if DEBUG_MAINLOOP_CHECK
    // Main loop time check
    uint32_t lt = micros();
    uint32_t Loop_Time = lt - startLoop;
    if (Loop_Time > Max_Loop_Time)
       Max_Loop_Time = Loop_Time;

    if (++Loop_cnt >= 1000)
       {
       Loop_cnt = 0;
       if (Max_Loop_Time_Disp == 0) Max_Loop_Time_Disp = Max_Loop_Time;
       Max_Loop_Time = 0;
       Avg_Loop_Time_Disp = lt - Avg_Loop_Start;
       Avg_Loop_Start = lt;
       }
  #endif // DEBUG_MAINLOOP_CHECK
}


/*
Der Sketch verwendet 27740 Bytes (90%) des Programmspeicherplatzes. Das Maximum sind 30720 Bytes.
Globale Variablen verwenden 1062 Bytes (51%) des dynamischen Speichers, 986 Bytes f√ºr lokale Variablen verbleiben. Das Maximum sind 2048 Bytes.

Der Sketch verwendet 27672 Bytes (90%) des Programmspeicherplatzes. Das Maximum sind 30720 Bytes.
Globale Variablen verwenden 1058 Bytes (51%) des dynamischen Speichers, 990 Bytes f√ºr lokale Variablen verbleiben. Das Maximum sind 2048 Bytes.
*/
