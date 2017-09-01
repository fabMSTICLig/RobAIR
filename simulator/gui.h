#ifndef GUI_H
#define GUI_H

struct gui_data_sources {
	struct servo *head;
};

int gui_init(void);
void gui_update(void);
void gui_deinit(void);

void gui_attach(struct gui_data_sources *srcs);

#endif
