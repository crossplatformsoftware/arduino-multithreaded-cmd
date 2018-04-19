// ---------------------------------------------------------------------------
// ARDUINO MULTI-THREADED COMMAND INTERPRETER
// ---------------------------------------------------------------------------
// Filename    : ArduinoThreadedCommandInterpreter.h
// Author      : UseLinuxNotWindows
// Version     : 1.0.0
// Target CPU  : ARM and AVR (supported by Arduino FreeRTOS)
// Toolset     : Arduino IDE or Eclipse Sloeber (http://eclipse.baeyens.it/)
// ---------------------------------------------------------------------------
// Needed to get around Sloeber bug
// ---------------------------------------------------------------------------

#ifndef ARDUINOTHREADEDCOMMANDINTERPRETER_H_
#define ARDUINOTHREADEDCOMMANDINTERPRETER_H_

typedef enum {
	INVALID_ITEM = 0,
	LED01 = 1,
	LED02 = 2,
	LED03 = 3,
	LED04 = 4,
	LED05 = 5,
	LED06 = 6,
	LED07 = 7,
	LED08 = 8
} Item_t;

typedef enum {
	INVALID_VALUE = 0,
	OFF   = 1,
	ON    = 2,
	STATE = 3
} Value_t;

typedef struct {
	Item_t  item;
	char *  item_text;
	Value_t value;
	char *  value_text;
} Command_t;

#endif /* ARDUINOTHREADEDCOMMANDINTERPRETER_H_ */

// ---------------------------------------------------------------------------
// END OF FILE
// ---------------------------------------------------------------------------
