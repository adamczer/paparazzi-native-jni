/*
 * Copyright (C) 2008-2010 The Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file firmwares/rotorcraft/main.h
 *
 * Rotorcraft main loop.
 */

#ifndef MAIN_H
#define MAIN_H

#ifdef SITL
#define STATIC_INLINE extern
#else
#define STATIC_INLINE static inline
#endif

STATIC_INLINE void main_init(void);
STATIC_INLINE void main_event(void);
STATIC_INLINE void handle_periodic_tasks(void);

STATIC_INLINE void main_periodic(void);
STATIC_INLINE void telemetry_periodic(void);
STATIC_INLINE void failsafe_check(void);

void main_event_juav();
//used to break apart down to the autopilot level
// drilling down to the guidance_h.c guidance_h_run(bool_t  in_flight)
void main_periodic_juav_autopilot_prior();
bool sys_time_check_and_ack_timer_main_periodic_juav();
void handle_periodic_tasks_following_main_periodic_juav();
void main_periodic_juav_autopilot_post();

void main_periodic_juav_test();


void main_event_juav_post(); //prior to autopilot rc
void main_event_juav_prior();//after autopilot rc


//JIVE

bool handel_periodic_task_modules_juav();
bool handel_periodic_task_radio_juav();
bool handel_periodic_task_failsafe_juav();
bool handel_periodic_task_electrical_juav();
bool handel_periodic_task_telemetry_juav();
bool handel_periodic_task_baro_juav();

#endif /* MAIN_H */
