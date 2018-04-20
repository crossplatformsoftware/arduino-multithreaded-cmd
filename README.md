# arduino-multithreaded-cmd

**Arduino Multi-Threaded Command Interpreter (Template) using FreeRTOS**

This code is designed as a template for implementing a command pattern. 
See "Gang of Four" Command Pattern https://en.wikipedia.org/wiki/Command_pattern

The Arduino waits for commands (`<ITEM>=<VALUE>` [Return]) sent via Serial,
interprets the command and sends the result (e.g. `<ITEM>=<VALUE>` [Return])
back via Serial.  This allows a single Arduino to control multiple items
individually (such as a bank of LED lights) but be controlled via a single
serial connection.

I used **Arduino FreeRTOS** https://github.com/feilipu/Arduino_FreeRTOS_Library
to implement a multi-threaded (time-sliced) architecture to minimise latency.
### 
                               +----------------------------------------------+
                               |                                              |
    +--------+ [Incoming Text] +-------------+ [Command Queue ] +-------------+
    |        | >>>>>>>>>>>>>>> | TaskReceive | >>>>>>>>>>>>>>>> | TaskRun     |
    | Client |   Serial Port   +-------------+ FreeRTOS Queues  | Command     |
    |        | <<<<<<<<<<<<<<< |  TaskSend   | <<<<<<<<<<<<<<<< | Interpreter |
    +--------+ [Outgoing Text] +-------------+ [Response Queue] +-------------+
                               |                                              |
                               +----------------------------------------------+
There are three tasks:

1. **TaskReceive** (serialEvent) - converts incoming characters into commands
2. **TaskRun** - receives commands, parses them, executes logic and sends result
3. **TaskSend** - receives result and sends the text back to the client

**TaskRun** and **TaskSend** could be combined into one process but this structure
allows the reply to be sent whilst **TaskRun** is executing the next command.
The code could be adapted to cases which require gradually changing values
(such as a window blind controller) by having a fourth worker task which
services the controller.

This demo code can be tested by connecting a serial terminal up to the
Arduino running at 115200 (no stop, 8 bits and no parity).

* Typing `LED01=ON` [Return]  -  should turn the on-board LED on
* Typing `LED01=OFF` [Return]  -  should turn the on-board LED on
* Typing `LED01=STATE` [Return]  -  should echo back the state of the on-board LED

I would strongly recommend Sloeber http://sloeber.io and https://github.com/Sloeber/arduino-eclipse-plugin 
over the standard Arduino IDE for building more sophisticated applications.

I created this multi-threaded sketch (template) in order to experiment with 
controlling multiple independent devices from a single Arduino from MQTT or 
OpenHAB.
