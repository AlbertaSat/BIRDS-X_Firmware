// main_loop_controller.c

#include "all_includes.h"
#include "main.h"

void run_once_on_boot() {
	
}

void main_loop() {
	DebugSerial.printf("Starting main_loop()\n");
	
	read_and_execute_next_incoming_command();

}

