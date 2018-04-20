// ---------------------------------------------------------------------------
// ARDUINO MULTI-THREADED COMMAND INTERPRETER (TEMPLATE)
// ---------------------------------------------------------------------------
// Filename    : ArduinoThreadedCommandInterpreter.ino
// Author      : crossplatformsoftware
// Version     : 1.0.0
// Target CPU  : AVR (Arduino FreeRTOS https://github.com/feilipu/avrfreertos)
// Toolset     : Arduino IDE or Eclipse Sloeber (http://eclipse.baeyens.it/)
// ---------------------------------------------------------------------------
//
// This code is designed as a template for implementing a command pattern.
// See "Gang of Four" "Command" https://en.wikipedia.org/wiki/Command_pattern
//
// The Arduino waits for commands (<ITEM>=<VALUE>[Return]) sent via Serial,
// interprets the command and sends the result (e.g. <ITEM>=<VALUE>[Return])
// back via Serial.  This allows a single Arduino to control multiple items
// individually (such as a bank of LED lights) but be controlled via a single
// serial connection.
//
// We use Arduino FreeRTOS implement a multi-threaded (time-sliced)
// architecture to minimise latency.
//
//                            +----------------------------------------------+
//                            |                                              |
// +--------+ [Incoming Text] +-------------+ [Command Queue ] +-------------+
// |        | >>>>>>>>>>>>>>> | TaskReceive | >>>>>>>>>>>>>>>> | TaskRun     |
// | Client |   Serial Port   +-------------+ FreeRTOS Queues  | Command     |
// |        | <<<<<<<<<<<<<<< |  TaskSend   | <<<<<<<<<<<<<<<< | Interpreter |
// +--------+ [Outgoing Text] +-------------+ [Response Queue] +-------------+
//                            |                                              |
//                            +----------------------------------------------+
// There are three tasks:
//
// 1) TaskReceive (serialEvent) - converts incoming characters into commands
// 2) TaskRun - receives commands, parses them, executes logic and sends result
// 3) TaskSend - receives result and sends the text back to the client
//
// TaskRun and TaskSend could be combined into one process but this structure
// allows the reply to be sent whilst TaskRun is executing the next command.
// The code could be adapted to cases which require gradually changing values
// (such as a window blind controller) by having a fourth worker task which
// services the controller.
//
// This demo code can be tested by connecting a serial terminal up to the
// Arduino running at 115200 (no stop, 8 bits and no parity).
//
// Typing LED01=ON[Return]    should turn the on-board LED on
// Typing LED01=OFF[Return]   should turn the on-board LED on
// Typing LED01=STATE[Return] should echo back the state of the on-board LED
//
// ---------------------------------------------------------------------------

#include <Arduino_FreeRTOS.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <src/queue.h>
#include <string.h>

#include "ArduinoThreadedCommandInterpreter.h"

// ---------------------------------------------------------------------------
// GLOBAL CONSTANTS
// ---------------------------------------------------------------------------

#define FIXED_SEND_SIZE

static const size_t QUEUE_LENGTH      = 4;
static const int    QUEUE_BUFFER_SIZE = 12;
static const long   BAUD_RATE         = 115200;
static const char   SEPARATOR         = '=';
static const char * SEP               = "=";
static const char * EOL               = "\r";

// ---------------------------------------------------------------------------
// GLOBAL VARIABLES - FreeRTOS queues
// ---------------------------------------------------------------------------

// FreeRTOS handles tricky things like mutexes and locking so we don't have to.
// The queue entries are fixed size buffers (12 characters in this case) which
// are easier to pass between tasks than variable length strings.

static QueueHandle_t rxQueue;  // Communication from TaskReceive to TaskCmd
static QueueHandle_t txQueue;  // Communication from TaskCmd to TaskSend

// ---------------------------------------------------------------------------
// SERIAL EVENT (TaskReceive) - receive interrupt driven by Arduino runtime
// ---------------------------------------------------------------------------

void serialEvent() /* TaskReceive */ {

	// This is effectively "TaskReceive" (driven by the serial port interrupt).
	// Its purpose is to split up the incoming serial stream into discrete
	// command messages ready for execution by the business logic.

	// The routine ignores input characters until we find the first 'visible'
	// character which is taken to be the start of the command. Characters
	// are stored until we reach end-of-line (CR/LF or 0x00) or we run out of
	// space at which point the message is queued for TaskRun to execute and
	// the receive buffer is cleared ready for the next message.

	// We are expecting an input message in the format "<COMMAND-MESSAGE>\n"
	// e.g. "LED01=ON\n" which we pass on to TaskRun via the FreeRTOS queue.

	// Static variables hold our state between successive calls to this routine
	static char    rxBuffer[QUEUE_BUFFER_SIZE];
	static char *  next_char = rxBuffer;
	static uint8_t bytes_left = QUEUE_BUFFER_SIZE;
	static bool    in_message = false;

	// Receive characters until the raw character queue is empty.
    while (Serial.available()) {

    	// Get next character from buffer
    	register char next = (char) Serial.read();

    	if (in_message) {

    		// If end-of-line (CR/LF/0x00) or no space left then queue message
    		if ( (next == '\r') ||
    		     (next == '\n') ||
				 (next == '\0') ||
				 (bytes_left == 1) ) {

    			// Clear rest of buffer to zero
    			memset((void *) next_char, 0, bytes_left);

    			// Queue message in receive queue - ignore errors
    			BaseType_t unused = pdFALSE;
    			xQueueSendFromISR(rxQueue, (const void *) rxBuffer, &unused);

    			// Clear receive buffer ready for next message
    			next_char = rxBuffer;
    			bytes_left = QUEUE_BUFFER_SIZE;
    			in_message = false;

    		} else {

    			// Otherwise just queue character in buffer
    			*next_char++ = next;
    			bytes_left--;
    		}

    	} else {

    		// Wait until we get a visible character before we begin
    		if (next > ' ') {
    			*next_char++ = next;
    			bytes_left--;
    			in_message = true;
    		}
    	}
    }
}

// ---------------------------------------------------------------------------
// PARSED COMMAND - parse the command string
// ---------------------------------------------------------------------------

static bool parsedCommand(Command_t & command, char * rxBuffer) {

	// TODO: Replace this code with your own command parser

	// Valid commands for this parser are:

	// LEDxx=ON[Return]    - Turn the LED ON  - returns state e.g. LED01=ON
	// LEDxx=OFF[Return]   - Turn the LED OFF - returns state e.g. LED01=OFF
	// LEDxx=STATE[Return] - Return the LED state e.g. LED01=ON

	// List of items that we understand.
	static const char * items[] = {

		"LED01",
		"LED02",
		"LED03",
		"LED04",
		"LED05",
		"LED06",
		"LED07",
		"LED08"
	};

	// List of values that we understand.
	static const char * values[] = {

		"OFF",
		"ON",
		"STATE"
	};

	char * value_text = NULL;

	// Initialise the command structure
	command.item = INVALID_ITEM;
	command.item_text = NULL;
	command.value = INVALID_VALUE;
	command.value_text = NULL;

	// Find the separator (e.g. "=") if we can
	for(int index = 0; index < QUEUE_BUFFER_SIZE; index++) {

		if (rxBuffer[index] == SEPARATOR) {
			value_text = &(rxBuffer[index]);
			*value_text++ = 0;
			break;
		}
	}

	// Recognise the item (LED01 to LED08)
	for(int index = LED01; index <= LED08; index++) {

		if (strcmp(rxBuffer, items[index - 1]) == 0) {

			command.item = (Item_t) index;
			command.item_text = (char *) items[index - 1];
			break;
		}
	}

	// Recognise the value - we could parse a number at this point
	if (value_text != NULL) {

		for(int index = OFF; index <= STATE; index++) {

			if (strcmp(value_text, values[index - 1]) == 0) {

				command.value = (Value_t) index;
				command.value_text = (char *) values[index - 1];
				break;
			}
		}
	}

	// Check to see if command is valid
	return ( (command.item  != INVALID_ITEM) &&
			 (command.value != INVALID_VALUE) );
}

// ---------------------------------------------------------------------------
// RUN COMMAND - perform the operation
// ---------------------------------------------------------------------------

static void RunCommand(Command_t & command, char * txBuffer) {

	// TODO: replace this code with your own business functionality

	// In the real case, this would be a bank of 8x LEDs connected through
	// solid state relays to the Arduino and we would be able to turn each
	// one ON/OFF individually.  In this demo code we just make use of the
	// built-in LED on the Arduino.

	// State of LED - either read or to write
	uint8_t led_state;

	// If the value is STATE then we are READING from the control port
	if (command.value == STATE) {

		// Find out the LED state
		switch(command.item) {

		case LED01:
			led_state = digitalRead(LED_BUILTIN);
			break;

		case LED02:
			led_state = digitalRead(LED_BUILTIN);
			break;

		case LED03:
			led_state = digitalRead(LED_BUILTIN);
			break;

		case LED04:
			led_state = digitalRead(LED_BUILTIN);
			break;

		case LED05:
			led_state = digitalRead(LED_BUILTIN);
			break;

		case LED06:
			led_state = digitalRead(LED_BUILTIN);
			break;

		case LED07:
			led_state = digitalRead(LED_BUILTIN);
			break;

		default: // LED08:
			led_state = digitalRead(LED_BUILTIN);
		}

		// Construct the reply (for STATE)
		strcpy(txBuffer, command.item_text);
		strcat(txBuffer, SEP);
		strcat(txBuffer, (led_state == HIGH) ? "ON" : "OFF");
		strcat(txBuffer, EOL);

	// Otherwise (ON/OFF) we are WRITING to the control port
	} else {

		// Value to set LED to
		led_state = (command.value == ON) ? HIGH : LOW;

		// Perform the business logic
		switch(command.item) {

		case LED01:
			digitalWrite(LED_BUILTIN, led_state);
			break;

		case LED02:
			digitalWrite(LED_BUILTIN, led_state);
			break;

		case LED03:
			digitalWrite(LED_BUILTIN, led_state);
			break;

		case LED04:
			digitalWrite(LED_BUILTIN, led_state);
			break;

		case LED05:
			digitalWrite(LED_BUILTIN, led_state);
			break;

		case LED06:
			digitalWrite(LED_BUILTIN, led_state);
			break;

		case LED07:
			digitalWrite(LED_BUILTIN, led_state);
			break;

		default: // LED08:
			digitalWrite(LED_BUILTIN, led_state);
		}

		// Construct a reply (for ON/OFF)
		strcpy(txBuffer, command.item_text);
		strcat(txBuffer, SEP);
		strcat(txBuffer, command.value_text);
		strcat(txBuffer, EOL);
	}
}

// ---------------------------------------------------------------------------
// ERROR HANDLER - handle an invalid command
// ---------------------------------------------------------------------------

static void ErrorHandler(Command_t & command, char * txBuffer) {

	// TODO: replace this code with your own error handler

	if (command.item == INVALID_ITEM)
		strcpy(txBuffer, "ERROR");
	else
		strcpy(txBuffer, command.item_text);

	strcat(txBuffer, SEP);
	strcat(txBuffer, "ERR");
	strcat(txBuffer, EOL);
}

// ---------------------------------------------------------------------------
// BUSINESS LOGIC - interpret the command
// ---------------------------------------------------------------------------

static void InterpretCommand(char * rxBuffer, char * txBuffer) {

	// Holds a parsed command
	Command_t command;

	// Clear our reply before we start
	memset((void *) txBuffer, 0, QUEUE_BUFFER_SIZE);

	// Attempt to parse the command
	if (parsedCommand(command, rxBuffer)) {

		// We have a valid command - so run it and generate reply
		RunCommand(command, txBuffer);

	} else {

		// Handle an invalid command by generating an error reply
		ErrorHandler(command, txBuffer);
	}
}

// ---------------------------------------------------------------------------
// TASK RUN - receives characters from stream and transmits command messages
// ---------------------------------------------------------------------------

void TaskRun(void *pvParameters) {

	// This task performs the business functionality of the application which
	// might be turning LEDs ON/OFF or reading LED values.  Commands are
	// queued up by TaskReceive (serialEvent) and replies sent to TaskSend.

	(void) pvParameters;

	static char rxBuffer[QUEUE_BUFFER_SIZE];  // Command from TaskReceive
	static char txBuffer[QUEUE_BUFFER_SIZE];  // Reply message to TaskSend

	// TASK LOOP - never exits
	for (;;) {

		// Code only runs when it receives a command from TaskReceive
		if (xQueueReceive(rxQueue, (void *) rxBuffer, portMAX_DELAY) == pdTRUE) {

			// Parse the command message, run the command and generate reply
			InterpretCommand(rxBuffer, txBuffer);

			// Send the reply to TaskSend - ignore queueing errors
			xQueueSend(txQueue, (void *) txBuffer, portMAX_DELAY);
		}
	}
}

// ---------------------------------------------------------------------------
// TASK SEND - Sends data back up the serial channel
// ---------------------------------------------------------------------------

void TaskSend(void *pvParameters) {

	// This task performs asynchronous writes to the serial channel sending
	// replies back after running commands.  This could be done from within
	// TaskCmd but having a separate task for this purpose allows the command
	// interpreter to begin the next command whilst sending the reply to the
	// previous one.

	(void) pvParameters;

	static char txBuffer[QUEUE_BUFFER_SIZE];  // Message from TaskRun

	// TASK LOOP - never exits
	for (;;) {

		// Code only runs when it receives a message from TaskRun
		if (xQueueReceive(txQueue, (void *) txBuffer, portMAX_DELAY) == pdTRUE) {

			// Just send it to the serial port
#ifdef FIXED_SEND_SIZE
			Serial.write(txBuffer, QUEUE_BUFFER_SIZE);
#else
			Serial.print(buffer);
#endif
		}
	}
}

// ---------------------------------------------------------------------------
// INITIALIZE HARDWARE - Set up any hardware connected to the Arduino
// ---------------------------------------------------------------------------

static void InitializeHardware() {

	// TODO: Set up your own hardware in here

	// In this demo code we just use the LED attached to the Arduino
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);
}

// ---------------------------------------------------------------------------
// INITIALIZE SERIAL - Set up the serial port
// ---------------------------------------------------------------------------

static void InitializeSerial() {

	// Set the baud rate
	Serial.begin(BAUD_RATE);

	// Wait for the port to become available
	while (!Serial) { }
}

// ---------------------------------------------------------------------------
// SETUP - called when you press reset or power the board
// ---------------------------------------------------------------------------

void setup() {

	static const configSTACK_DEPTH_TYPE STACK_SIZE = 128;

	static const portCHAR * TASKCMD  = "TaskCmd";
	static const portCHAR * TASKSEND = "TaskSend";

	static const UBaseType_t LO_PRIORITY = 1;
	static const UBaseType_t HI_PRIORITY = 2;

	// Create the global FreeRTOS queues (each holds 4 x 12 character buffers)
	rxQueue = xQueueCreate(QUEUE_LENGTH, QUEUE_BUFFER_SIZE);
	txQueue = xQueueCreate(QUEUE_LENGTH, QUEUE_BUFFER_SIZE);

	// Initialize the hardware
	InitializeHardware();

	// Create the TaskReceive task (set up the serial port)
	InitializeSerial();

	// Create the TaskRun and TaskSend tasks (as FreeRTOS tasks)
	xTaskCreate(TaskRun,  TASKCMD,  STACK_SIZE, NULL, LO_PRIORITY, NULL);
	xTaskCreate(TaskSend, TASKSEND, STACK_SIZE, NULL, HI_PRIORITY, NULL);

	// FreeRTOS starts after this point
}

// ---------------------------------------------------------------------------
// IDLE LOOP - loop() is used as the idle loop in FreeRTOS
// ---------------------------------------------------------------------------

void loop()
{
	// Used as an idle loop within FreeRTOS - we use it to sleep to save power

	// Now is the time to go to sleep. In the Atmega8 datasheet
	// http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page
	// 35 there is a list of sleep modes which explains which clocks and
	// wake up sources are available in which sleep modus.
	//
	// Sleep modules are found in avr/sleep.h file:
	// The 5 different modes are:
	//
	// SLEEP_MODE_IDLE -the least power savings
	// SLEEP_MODE_ADC
	// SLEEP_MODE_PWR_SAVE
	// SLEEP_MODE_STANDBY
	// SLEEP_MODE_PWR_DOWN -the most power savings
	//
	// The power reduction management <avr/power.h> is described in
	// http://www.nongnu.org/avr-libc/user-manual/group_avr_power.html

	// Necessary for some reason
	delay(100);

	// Sleep mode is set here
	set_sleep_mode(SLEEP_MODE_IDLE);

	// Enable the sleep bit in the MCUCR register so sleep is possible.
	sleep_enable();

	//  Power down most peripherals except for the GPIO pins
	power_adc_disable();
	power_spi_disable();
	power_timer0_disable();
	power_timer1_disable();
	power_timer2_disable();
	power_twi_disable();

	// Go to SLEEP
	sleep_mode();

	// THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
	sleep_disable();

	// disable sleep...
	power_all_enable();
}

// ---------------------------------------------------------------------------
// END OF FILE
// ---------------------------------------------------------------------------
