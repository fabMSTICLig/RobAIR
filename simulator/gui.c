#include "gui.h"

#include <math.h>
#include <signal.h>
#include <sys/time.h>
#include <time.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL2_gfxPrimitives.h>

#include "servo.h"

#define REFRESH_MS 50

#define DEG_TO_RAD(x) ((double)(x) * M_PI / 180.)

#define WIN_WIDTH  500
#define WIN_HEIGHT 700

#define HEAD_CENTER_X  (WIN_WIDTH / 2)
#define HEAD_CENTER_Y  (2 * WIN_HEIGHT / 3)
#define HEAD_RADIUS    50
#define HEAD_ARROW_LEN 60

#define EYES_ROWS  5
#define EYES_COLS  14
#define EYES_NUM   (EYES_ROWS * EYES_COLS)
#define EYES_TOP_Y 100

#define SPEED_LEFT_X        (HEAD_CENTER_X - HEAD_ARROW_LEN - 30)
#define SPEED_RIGHT_X       (HEAD_CENTER_X + HEAD_ARROW_LEN + 30)
#define SPEED_Y             HEAD_CENTER_Y
#define SPEED_LEN_MAX       100
#define SPEED_ARROWHEAD_LEN 5

SDL_Window *win = NULL;
SDL_Renderer *ren = NULL;
SDL_Texture *canvas = NULL;

int needs_redrawing = 0;

int head_angle = 0;
int eyes_leds = 0;
struct ws2812_color eyes_colors[EYES_NUM];
int8_t speed_left, speed_right;


static void on_timer(union sigval);


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

static void gui_draw_eyes()
{
	int i = 0;
	int pos_y = EYES_TOP_Y;

	for (int y = 0 ; y < EYES_ROWS ; ++y) {
		int pos_x;
		if (y % 2)
			pos_x = (WIN_WIDTH / 2) + ((EYES_COLS - 1) * 30 / 2);
		else
			pos_x = (WIN_WIDTH / 2) - ((EYES_COLS - 1) * 30 / 2);

		for (int x = 0 ; x < EYES_COLS ; ++x) {
			unsigned int r, g, b;
			r = eyes_colors[i].r * 6;
			g = eyes_colors[i].g * 6;
			b = eyes_colors[i].b * 6;

			if (r != 0 || g != 0 || b != 0) {
				if (r > 255)
					r = 255;
				if (g > 255)
					g = 255;
				if (b > 255)
					b = 255;

				filledCircleRGBA(ren, pos_x, pos_y, 10,
						r, g, b, 0xff);
				circleRGBA(ren, pos_x, pos_y, 10,
						0, 0, 0, 0xff);
			}

			i++;
			pos_x += (y % 2) ? -30 : 30;
		}

		pos_y += 25;
	}
}

static void gui_draw_speed()
{
	double len_left = speed_left * SPEED_LEN_MAX / INT8_MAX,
	       len_right = speed_right * SPEED_LEN_MAX / INT8_MAX;

	SDL_RenderDrawLine(ren, SPEED_LEFT_X, SPEED_Y,
			SPEED_LEFT_X, SPEED_Y - len_left);
	SDL_RenderDrawLine(ren, SPEED_RIGHT_X, SPEED_Y,
			SPEED_RIGHT_X, SPEED_Y - len_right);

	double head_len;

	if (len_left < SPEED_ARROWHEAD_LEN * 2
			&& len_left > -SPEED_ARROWHEAD_LEN * 2)
		head_len = -len_left / 2;
	else if (len_left > 0)
		head_len = -SPEED_ARROWHEAD_LEN;
	else
		head_len = SPEED_ARROWHEAD_LEN;

	SDL_RenderDrawLine(ren, SPEED_LEFT_X, SPEED_Y - len_left,
			SPEED_LEFT_X - head_len,
			SPEED_Y - len_left - head_len);

	SDL_RenderDrawLine(ren, SPEED_LEFT_X, SPEED_Y - len_left,
			SPEED_LEFT_X + head_len,
			SPEED_Y - len_left - head_len);


	if (len_right < SPEED_ARROWHEAD_LEN * 2
			&& len_right > -SPEED_ARROWHEAD_LEN * 2)
		head_len = -len_right / 2;
	else if (len_right > 0)
		head_len = -SPEED_ARROWHEAD_LEN;
	else
		head_len = SPEED_ARROWHEAD_LEN;

	SDL_RenderDrawLine(ren, SPEED_RIGHT_X, SPEED_Y - len_right,
			SPEED_RIGHT_X - head_len,
			SPEED_Y - len_right - head_len);

	SDL_RenderDrawLine(ren, SPEED_RIGHT_X, SPEED_Y - len_right,
			SPEED_RIGHT_X + head_len,
			SPEED_Y - len_right - head_len);
}

static void gui_clear()
{
	boxRGBA(ren, 0, 0, WIN_WIDTH, WIN_HEIGHT, 0xff, 0xff, 0xff, 0xff);
}

static void gui_draw()
{
	gui_clear();
	gui_draw_head();
	gui_draw_eyes();
	gui_draw_speed();
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

	memset(eyes_colors, 0, EYES_NUM * sizeof(*eyes_colors));

	gui_draw();
	gui_display();

	struct sigevent alarm_action = {
		.sigev_notify = SIGEV_THREAD,
		.sigev_notify_function = on_timer
	};

	struct itimerspec update_period = {
		.it_interval = {
			.tv_sec = 0,
			.tv_nsec = REFRESH_MS * 1000000
		},
		.it_value = {
			.tv_sec = 0,
			.tv_nsec = REFRESH_MS * 1000000
		}
	};

	timer_t timer;
	timer_create(CLOCK_REALTIME, &alarm_action, &timer);
	timer_settime(timer, 0, &update_period, NULL);

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

		SDL_Quit();
	}
}

int gui_is_active()
{
	return win != NULL;
}

static void gui_head_cb(int angle)
{
	head_angle = angle;
	needs_redrawing = 1;
}

static void gui_eyes_cb(unsigned int num_leds, struct ws2812_color *colors)
{
	if (num_leds > EYES_NUM)
		num_leds = EYES_NUM;

	unsigned int rem_leds = EYES_NUM - num_leds;

	memcpy(eyes_colors, colors, num_leds * sizeof(*colors));
	memset(eyes_colors + num_leds, 0, rem_leds * sizeof(*colors));

	needs_redrawing = 1;
}

static void gui_motors_cb(int8_t speed1, int8_t speed2)
{
	speed_left = speed1;
	speed_right = speed2;
	needs_redrawing = 1;
}

void gui_attach(struct gui_data_sources *srcs)
{
	if (srcs->head != NULL)
		servo_set_callback(srcs->head, gui_head_cb);
	if (srcs->eyes != NULL)
		ws2812_set_callback(srcs->eyes, gui_eyes_cb);
	if (srcs->motors != NULL)
		md49_set_callback(srcs->motors, gui_motors_cb);
}


static int updating = 0;

static void on_timer(union sigval val)
{
	if (updating)
		return;
	updating = 1;
	gui_update();
	updating = 0;
}
