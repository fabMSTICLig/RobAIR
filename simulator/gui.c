#include "gui.h"

#include <math.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL2_gfxPrimitives.h>

#include "servo.h"

#define DEG_TO_RAD(x) ((double)(x) * M_PI / 180.)

#define WIN_WIDTH  500
#define WIN_HEIGHT 700

#define HEAD_CENTER_X  (WIN_WIDTH / 2)
#define HEAD_CENTER_Y  (WIN_HEIGHT / 2)
#define HEAD_RADIUS    50
#define HEAD_ARROW_LEN 60

SDL_Window *win = NULL;
SDL_Renderer *ren = NULL;
SDL_Texture *canvas = NULL;

int needs_redrawing = 0;

int head_angle = 0;


static void gui_draw_head()
{
	circleRGBA(ren, HEAD_CENTER_X, HEAD_CENTER_Y, HEAD_RADIUS,
			0, 0, 0, 0xff);

	double arrow_head[2];
	arrow_head[0] = HEAD_CENTER_X
		+ cos(DEG_TO_RAD(180 - head_angle)) * HEAD_ARROW_LEN;
	arrow_head[1] = HEAD_CENTER_Y
		- sin(DEG_TO_RAD(180 - head_angle)) * HEAD_ARROW_LEN;

	SDL_RenderDrawLine(ren, HEAD_CENTER_X, HEAD_CENTER_Y,
			arrow_head[0], arrow_head[1]);
}

static void gui_clear()
{
	boxRGBA(ren, 0, 0, WIN_WIDTH, WIN_HEIGHT, 0xff, 0xff, 0xff, 0xff);
}

static void gui_draw()
{
	gui_clear();
	gui_draw_head();
	SDL_RenderPresent(ren);
}

static void gui_display()
{
	SDL_SetRenderTarget(ren, NULL);
	SDL_RenderCopy(ren, canvas, NULL, NULL);
	SDL_RenderPresent(ren);
	SDL_SetRenderTarget(ren, canvas);
}

int gui_init()
{
	if (SDL_Init(SDL_INIT_VIDEO))
		return 1;

	if (SDL_CreateWindowAndRenderer(
			WIN_WIDTH,
			WIN_HEIGHT,
			0,
			&win,
			&ren)) {
		SDL_Quit();
		return 1;
	}

	SDL_SetWindowTitle(win, "Robairsim");

	canvas = SDL_CreateTexture(ren, SDL_PIXELFORMAT_RGBA8888,
			SDL_TEXTUREACCESS_TARGET, WIN_WIDTH, WIN_HEIGHT);
	SDL_SetRenderTarget(ren, canvas);

	gui_draw();
	gui_display();

	return 0;
}

void gui_update()
{
	if (win == NULL)
		return;

	SDL_Event event;

	while (SDL_PollEvent(&event)) {
		switch (event.type) {
		case SDL_WINDOWEVENT:
			if (event.window.event == SDL_WINDOWEVENT_EXPOSED &&
					!needs_redrawing)
				gui_display();
			break;
		case SDL_QUIT:
			gui_deinit();
			return;
		}
	}

	if (needs_redrawing) {
		gui_draw();
		gui_display();
		needs_redrawing = 0;
	}
}

void gui_deinit()
{
	if (ren != NULL) {
		SDL_DestroyRenderer(ren);
		ren = NULL;
	}

	if (win != NULL) {
		SDL_DestroyWindow(win);
		win = NULL;
	}

	SDL_Quit();
}

static void gui_head_cb(int angle)
{
	head_angle = angle;
	needs_redrawing = 1;
}

void gui_attach(struct gui_data_sources *srcs)
{
	if (srcs->head != NULL)
		servo_set_callback(srcs->head, gui_head_cb);
}
