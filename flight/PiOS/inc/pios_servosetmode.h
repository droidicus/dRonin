/**
 ******************************************************************************
 * @addtogroup PIOS PIOS SetMode hardware abstraction layer
 * @{
 * @addtogroup   PIOS_SERVO RC Servo Functions
 * @{
 *
 * @file       pios_servosetmode.h  
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2015
 * @author     dRonin, http://dronin.org Copyright (C) 2015
 * @brief      RC Servo SetMode functions header.
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

#ifndef PIOS_SERVOSETMODE_H
#define PIOS_SERVOSETMODE_H

/* Public Functions */
extern void PIOS_Servo_SetMode(const uint16_t * update_rates, const enum pwm_mode *pwm_mdoe, uint8_t banks);

#endif /* PIOS_SERVOSETMODE_H */

/**
  * @}
  * @}
  */
