/*
 * Copyright © 2014-2015 Red Hat, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

/*
 * TOUCHPAD EDGE MOTION
 *
 * This module implements automatic cursor motion when performing tap-and-drag
 * operations near the edges of a touchpad. When a user starts dragging content
 * and reaches the edge of the touchpad, the system automatically continues
 * moving the cursor in that direction to allow selection/dragging of content
 * that extends beyond the physical touchpad boundaries.
 */

#include "config.h"

#include <math.h>
#include <string.h>

#include "evdev-mt-touchpad.h"

void
tp_edge_motion_init(struct tp_dispatch *tp);

enum edge_motion_state { STATE_IDLE, STATE_DRAG_ACTIVE, STATE_EDGE_MOTION };

struct edge_motion_fsm {
	enum edge_motion_state current_state;
	uint64_t last_motion_time;
	uint32_t current_edge;
	double motion_dx;
	double motion_dy;
	uint64_t continuous_motion_count;
	struct tp_dispatch *tp;
	struct libinput_timer timer;
};

static struct edge_motion_fsm fsm = {
	.current_state = STATE_IDLE,
	.tp = NULL,
};

#define EDGE_MOTION_CONFIG_SPEED_MM_S		70.0
#define EDGE_MOTION_CONFIG_MIN_INTERVAL_US	8000
#define EDGE_MOTION_CONFIG_EDGE_THRESHOLD_MM	5.0

static void
calculate_motion_vector(uint32_t edge, double *dx, double *dy)
{
	*dx = 0.0;
	*dy = 0.0;

	if (edge & EDGE_LEFT)
		*dx = -1.0;
	else if (edge & EDGE_RIGHT)
		*dx = 1.0;

	if (edge & EDGE_TOP)
		*dy = -1.0;
	else if (edge & EDGE_BOTTOM)
		*dy = 1.0;

	double mag = sqrt((*dx) * (*dx) + (*dy) * (*dy));
	if (mag > 0) {
		*dx /= mag;
		*dy /= mag;
	}
}

static void
inject_accumulated_motion(struct tp_dispatch *tp, uint64_t time)
{
	if (fsm.last_motion_time == 0) {
		fsm.last_motion_time = time;
		return;
	}

	uint64_t time_since_last = time - fsm.last_motion_time;
	double dist_mm =
		EDGE_MOTION_CONFIG_SPEED_MM_S * ((double)time_since_last / 1000000.0);

	if (dist_mm < 0.001)
		return;

	struct device_float_coords raw = {
		.x = fsm.motion_dx * dist_mm * tp->accel.x_scale_coeff,
		.y = fsm.motion_dy * dist_mm * tp->accel.y_scale_coeff
	};

	struct normalized_coords delta =
		filter_dispatch(tp->device->pointer.filter, &raw, tp, time);

	pointer_notify_motion(&tp->device->base, time, &delta, &raw);

	fsm.last_motion_time = time;
	fsm.continuous_motion_count++;
}

static uint32_t
detect_touch_edge(const struct tp_dispatch *tp, const struct tp_touch *t)
{
	uint32_t edge = EDGE_NONE;
	struct phys_coords mm = { EDGE_MOTION_CONFIG_EDGE_THRESHOLD_MM,
				  EDGE_MOTION_CONFIG_EDGE_THRESHOLD_MM };
	struct device_coords threshold = evdev_device_mm_to_units(tp->device, &mm);

	if (t->point.x < threshold.x)
		edge |= EDGE_LEFT;
	if (t->point.x > tp->device->abs.absinfo_x->maximum - threshold.x)
		edge |= EDGE_RIGHT;
	if (t->point.y < threshold.y)
		edge |= EDGE_TOP;
	if (t->point.y > tp->device->abs.absinfo_y->maximum - threshold.y)
		edge |= EDGE_BOTTOM;

	return edge;
}

static void
tp_edge_motion_handle_timeout(uint64_t now, void *data)
{
	struct edge_motion_fsm *fsm_ptr = data;

	if (fsm_ptr->current_state != STATE_EDGE_MOTION)
		return;

	inject_accumulated_motion(fsm_ptr->tp, now);
	libinput_timer_set(&fsm_ptr->timer, now + EDGE_MOTION_CONFIG_MIN_INTERVAL_US);
}

void
tp_edge_motion_init(struct tp_dispatch *tp)
{
	if (fsm.tp)
		return;

	memset(&fsm, 0, sizeof(fsm));
	fsm.current_state = STATE_IDLE;
	fsm.tp = tp;

	libinput_timer_init(&fsm.timer,
			    tp_libinput_context(tp),
			    "edge drag motion",
			    tp_edge_motion_handle_timeout,
			    &fsm);
}

void
tp_edge_motion_cleanup(void)
{
	if (fsm.tp)
		libinput_timer_destroy(&fsm.timer);

	memset(&fsm, 0, sizeof(fsm));
	fsm.current_state = STATE_IDLE;
}

int
tp_edge_motion_handle_drag_state(struct tp_dispatch *tp, uint64_t time)
{
	if (!fsm.tp)
		tp_edge_motion_init(tp);

	bool drag_active = false;

	switch (tp->tap.state) {
	case TAP_STATE_1FGTAP_DRAGGING:
	case TAP_STATE_1FGTAP_DRAGGING_2:
	case TAP_STATE_1FGTAP_DRAGGING_WAIT:
	case TAP_STATE_1FGTAP_DRAGGING_OR_TAP:
	case TAP_STATE_1FGTAP_DRAGGING_OR_DOUBLETAP:
		drag_active = true;
		break;
	default:
		drag_active = false;
		break;
	}

	uint32_t detected_edge = EDGE_NONE;
	struct tp_touch *t;

	if (drag_active) {
		tp_for_each_touch(tp, t) {
			if (t->state != TOUCH_NONE && t->state != TOUCH_HOVERING) {
				detected_edge = detect_touch_edge(tp, t);
				break;
			}
		}
	}

	enum edge_motion_state next_state = STATE_IDLE;
	if (drag_active) {
		next_state = (detected_edge != EDGE_NONE) ? STATE_EDGE_MOTION
							  : STATE_DRAG_ACTIVE;
	}

	if (next_state != fsm.current_state) {
		fsm.current_state = next_state;
		fsm.current_edge = detected_edge;

		if (fsm.current_state != STATE_EDGE_MOTION)
			fsm.continuous_motion_count = 0;

		switch (fsm.current_state) {
		case STATE_IDLE:
		case STATE_DRAG_ACTIVE:
			libinput_timer_cancel(&fsm.timer);
			break;

		case STATE_EDGE_MOTION:
			calculate_motion_vector(fsm.current_edge,
						&fsm.motion_dx,
						&fsm.motion_dy);
			fsm.last_motion_time = time;
			tp_edge_motion_handle_timeout(time, &fsm);
			break;
		}
	} else if (fsm.current_state == STATE_EDGE_MOTION &&
		   detected_edge != fsm.current_edge) {
		fsm.current_edge = detected_edge;
		calculate_motion_vector(fsm.current_edge,
					&fsm.motion_dx,
					&fsm.motion_dy);
	}

	return (fsm.current_state == STATE_EDGE_MOTION);
}
