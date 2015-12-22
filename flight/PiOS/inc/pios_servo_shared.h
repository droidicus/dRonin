/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup   PIOS_SERVO Servo Functions
 * @brief PIOS interface to read and write from servo PWM ports
 * @{
 *
 * @file       pios_servo_shared.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     dRonin, http://dronin.org Copyright (C) 2015
 * @brief      Servo shared structures.
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef PIOS_SERVO_SHARED_H
#define PIOS_SERVO_SHARED_H

/* Private Function Prototypes */
extern const struct pios_servo_cfg * servo_cfg;

//! The counter rate for the channel, used to calculate compare values.
extern enum pwm_mode *output_channel_resolution;  // The clock rate for that timer
#if defined(PIOS_INCLUDE_HPWM)
enum SYNC_PWM {SYNC_PWM_FALSE, SYNC_PWM_TRUE};
extern enum SYNC_PWM *output_channel_mode;
#endif

#endif /* PIOS_SERVO_SHARED_H */

/**
 * @}
 * @}
 */
