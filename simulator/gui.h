#ifndef GUI_H
#define GUI_H

#include "servo.h"
#include "ws2812.h"

struct gui_data_sources {
	struct servo *head;
	struct ws2812 *eyes;
};

int gui_init(void);
void gui_update(void);
void gui_deinit(void);
int gui_is_active(void);

void gui_attach(struct gui_data_sources *srcs);

#endif
