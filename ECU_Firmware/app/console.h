/* Text console on the USB port, for checking the ECU on the bench. */
#ifndef CONSOLE_H
#define CONSOLE_H

void console_init(void);
void console_poll(void); // reads and runs typed commands
void console_tick(void); // every millisecond, for the status stream

#endif
