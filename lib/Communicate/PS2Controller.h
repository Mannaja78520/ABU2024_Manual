#ifndef PS2CONTROLLER_H
#define PS2CONTROLLER_H

#include <DefinePin.h>

#include <DigitalIO.h>
#include <PsxControllerBitBang.h>

#include <avr/pgmspace.h>
typedef const __FlashStringHelper * FlashStr;
typedef const byte* PGM_BYTES_P;
#define PSTR_TO_F(s) reinterpret_cast<const __FlashStringHelper *> (s)

const byte PIN_BUTTONPRESS = A0;
const byte PIN_HAVECONTROLLER = A1;

const char buttonSelectName[] PROGMEM = "Select";
const char buttonL3Name[] PROGMEM = "L3";
const char buttonR3Name[] PROGMEM = "R3";
const char buttonStartName[] PROGMEM = "Start";
const char buttonUpName[] PROGMEM = "Up";
const char buttonRightName[] PROGMEM = "Right";
const char buttonDownName[] PROGMEM = "Down";
const char buttonLeftName[] PROGMEM = "Left";
const char buttonL2Name[] PROGMEM = "L2";
const char buttonR2Name[] PROGMEM = "R2";
const char buttonL1Name[] PROGMEM = "L1";
const char buttonR1Name[] PROGMEM = "R1";
const char buttonTriangleName[] PROGMEM = "Triangle";
const char buttonCircleName[] PROGMEM = "Circle";
const char buttonCrossName[] PROGMEM = "Cross";
const char buttonSquareName[] PROGMEM = "Square";

const char* const psxButtonNames[PSX_BUTTONS_NO] PROGMEM = {
	buttonSelectName,
	buttonL3Name,
	buttonR3Name,
	buttonStartName,
	buttonUpName,
	buttonRightName,
	buttonDownName,
	buttonLeftName,
	buttonL2Name,
	buttonR2Name,
	buttonL1Name,
	buttonR1Name,
	buttonTriangleName,
	buttonCircleName,
	buttonCrossName,
	buttonSquareName
};

byte psxButtonToIndex (PsxButtons psxButtons) {
	byte i;

	for (i = 0; i < PSX_BUTTONS_NO; ++i) {
		if (psxButtons & 0x01) {
			break;
		}

		psxButtons >>= 1U;
	}

	return i;
}

FlashStr getButtonName (PsxButtons psxButton) {
	FlashStr ret = F("");
	
	byte b = psxButtonToIndex (psxButton);
	if (b < PSX_BUTTONS_NO) {
		PGM_BYTES_P bName = reinterpret_cast<PGM_BYTES_P> (pgm_read_ptr (&(psxButtonNames[b])));
		ret = PSTR_TO_F (bName);
	}

	return ret;
}

void dumpButtons (PsxButtons psxButtons) {
	static PsxButtons lastB = 0;

	if (psxButtons != lastB) {
		lastB = psxButtons;     // Save it before we alter it
		
		Serial.print (F("Pressed: "));

		for (byte i = 0; i < PSX_BUTTONS_NO; ++i) {
			byte b = psxButtonToIndex (psxButtons);
			if (b < PSX_BUTTONS_NO) {
				PGM_BYTES_P bName = reinterpret_cast<PGM_BYTES_P> (pgm_read_ptr (&(psxButtonNames[b])));
				Serial.print (PSTR_TO_F (bName));
			}

			psxButtons &= ~(1 << b);

			if (psxButtons != 0) {
				Serial.print (F(", "));
			}
		}

		Serial.println ();
	}
}

void dumpAnalog (const char *str, const byte x, const byte y) {
	Serial.print (str);
	Serial.print (F(" analog: x = "));
	Serial.print (x);
	Serial.print (F(", y = "));
	Serial.println (y);
}



const char ctrlTypeUnknown[] PROGMEM = "Unknown";
const char ctrlTypeDualShock[] PROGMEM = "Dual Shock";
const char ctrlTypeDsWireless[] PROGMEM = "Dual Shock Wireless";
const char ctrlTypeGuitHero[] PROGMEM = "Guitar Hero";
const char ctrlTypeOutOfBounds[] PROGMEM = "(Out of bounds)";

const char* const controllerTypeStrings[PSCTRL_MAX + 1] PROGMEM = {
	ctrlTypeUnknown,
	ctrlTypeDualShock,
	ctrlTypeDsWireless,
	ctrlTypeGuitHero,
	ctrlTypeOutOfBounds
};

PsxControllerBitBang<PIN_PS2_ATT, PIN_PS2_CMD, PIN_PS2_DAT, PIN_PS2_CLK> psx;

boolean haveController = false;
 

class Joystick{
    public:
        void init(){
            Serial.begin (115200);
            Serial.println (F("Ready!"));
        }

        void Loop(){

            fastDigitalWrite (PIN_HAVECONTROLLER, haveController);
            
            if (!haveController) {
                if (psx.begin ()) {
                    Serial.println (F("Controller found!"));
                    delay (300);
                    if (!psx.enterConfigMode ()) {
                        Serial.println (F("Cannot enter config mode"));
                    } else {
                        PsxControllerType ctype = psx.getControllerType ();
                        PGM_BYTES_P cname = reinterpret_cast<PGM_BYTES_P> (pgm_read_ptr (&(controllerTypeStrings[ctype < PSCTRL_MAX ? static_cast<byte> (ctype) : PSCTRL_MAX])));
                        Serial.print (F("Controller Type is: "));
                        Serial.println (PSTR_TO_F (cname));

                        if (!psx.enableAnalogSticks ()) {
                            Serial.println (F("Cannot enable analog sticks"));
                        }
                        
                        if (!psx.enableAnalogButtons ()) {
                            Serial.println (F("Cannot enable analog buttons"));
                        }
                        
                        if (!psx.exitConfigMode ()) {
                            Serial.println (F("Cannot exit config mode"));
                        }
                    }
                    
                    haveController = true;
                }
            } else {
                if (!psx.read ()) {
                    Serial.println (F("Controller lost :("));
                    haveController = false;
                }
            }
        }

        void LoopTest(){
            static byte slx, sly, srx, sry;
            
            fastDigitalWrite (PIN_HAVECONTROLLER, haveController);
            
            if (!haveController) {
                if (psx.begin ()) {
                    Serial.println (F("Controller found!"));
                    delay (300);
                    if (!psx.enterConfigMode ()) {
                        Serial.println (F("Cannot enter config mode"));
                    } else {
                        PsxControllerType ctype = psx.getControllerType ();
                        PGM_BYTES_P cname = reinterpret_cast<PGM_BYTES_P> (pgm_read_ptr (&(controllerTypeStrings[ctype < PSCTRL_MAX ? static_cast<byte> (ctype) : PSCTRL_MAX])));
                        Serial.print (F("Controller Type is: "));
                        Serial.println (PSTR_TO_F (cname));

                        if (!psx.enableAnalogSticks ()) {
                            Serial.println (F("Cannot enable analog sticks"));
                        }
                        
                        if (!psx.enableAnalogButtons ()) {
                            Serial.println (F("Cannot enable analog buttons"));
                        }
                        
                        if (!psx.exitConfigMode ()) {
                            Serial.println (F("Cannot exit config mode"));
                        }
                    }
                    
                    haveController = true;
                }
            } else {
                if (!psx.read ()) {
                    Serial.println (F("Controller lost :("));
                    haveController = false;
                } else {
                    fastDigitalWrite (PIN_BUTTONPRESS, !!psx.getButtonWord ());
                    dumpButtons (psx.getButtonWord ());

                    byte lx, ly;
                    psx.getLeftAnalog (lx, ly);
                    if (lx != slx || ly != sly) {
                        dumpAnalog ("Left", lx, ly);
                        slx = lx;
                        sly = ly;
                    }

                    byte rx, ry;
                    psx.getRightAnalog (rx, ry);
                    if (rx != srx || ry != sry) {
                        dumpAnalog ("Right", rx, ry);
                        srx = rx;
                        sry = ry;
                    }
                }
            }

            delay (1000 / 60);
        }

};

#endif