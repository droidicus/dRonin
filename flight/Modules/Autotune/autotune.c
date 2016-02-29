/**
 ******************************************************************************
 * @addtogroup TauLabsModules Tau Labs Modules
 * @{
 * @addtogroup AutotuningModule Autotuning Module
 * @{
 *
 * @file       autotune.c
 * @author     dRonin, http://dRonin.org/, Copyright (C) 2015-2016
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013-2014
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
 * @brief      State machine to run autotuning. Low level work done by @ref
 *             StabilizationModule
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 ******************************************************************************/
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
 *
 * Additional note on redistribution: The copyright and license notices above
 * must be maintained in each individual source file that is a derivative work
 * of this source file; otherwise redistribution is prohibited.
 */

#include "openpilot.h"
#include "pios.h"
#include "physical_constants.h"
#include "flightstatus.h"
#include "modulesettings.h"
#include "manualcontrolcommand.h"
#include "manualcontrolsettings.h"
#include "gyros.h"
#include "actuatordesired.h"
#include "stabilizationdesired.h"
#include "stabilizationsettings.h"
#include "systemident.h"
#include <pios_board_info.h>
#include "pios_thread.h"
#include "systemsettings.h"

#include "circqueue.h"
#include "misc_math.h"

// Private constants
#define STACK_SIZE_BYTES 21340
#define TASK_PRIORITY PIOS_THREAD_PRIO_NORMAL
#define AUTOTUNE_CYCLE_LENGTH 600
#define NUM_UAVO_ELEM 6

// Private types
enum AUTOTUNE_STATE { AT_INIT, AT_START, AT_RUN, AT_FINISHED, AT_WAITING };

struct at_queued_data {
	float y[3];		/* Gyro measurements */
	float u[3];		/* Actuator desired */
	float thrust;	/* thrust desired */

	float dT;	/* From Gyro */
	uint32_t ticks; /* From Gyro */
};

// Private variables
static struct pios_thread *taskHandle;
static bool module_enabled;
static circ_queue_t at_queue;
static volatile uint32_t at_points_spilled;
static uint32_t thrust_accumulator;
static uint16_t log_index;

// Private functions
static void AutotuneTask(void *parameters);
static void af_predict(float gyroAccum[AUTOTUNE_CYCLE_LENGTH][3], float actuatorAccum[AUTOTUNE_CYCLE_LENGTH][3],
						float thrustAccum[AUTOTUNE_CYCLE_LENGTH], const float u_in[3], const float gyro[3],
						const uint32_t ticks, const float t_in);
static void af_init();

#ifndef AT_QUEUE_NUMELEM
#define AT_QUEUE_NUMELEM 18
#endif

/**
 * Initialise the module, called on startup
 * \returns 0 on success or -1 if initialisation failed
 */
int32_t AutotuneInitialize(void)
{
	// Create a queue, connect to manual control command and flightstatus
#ifdef MODULE_Autotune_BUILTIN
	module_enabled = true;
#else
	uint8_t module_state[MODULESETTINGS_ADMINSTATE_NUMELEM];
	ModuleSettingsAdminStateGet(module_state);
	if (module_state[MODULESETTINGS_ADMINSTATE_AUTOTUNE] == MODULESETTINGS_ADMINSTATE_ENABLED)
		module_enabled = true;
	else
		module_enabled = false;
#endif

	if (module_enabled) {
		SystemIdentInitialize();

		at_queue = circ_queue_new(sizeof(struct at_queued_data),
				AT_QUEUE_NUMELEM);

		if (!at_queue)
			module_enabled = false;
	}

	log_index = 0;

	return 0;
}

/**
 * Initialise the module, called on startup
 * \returns 0 on success or -1 if initialisation failed
 */
int32_t AutotuneStart(void)
{
	// Start main task if it is enabled
	if(module_enabled) {
		taskHandle = PIOS_Thread_Create(AutotuneTask, "Autotune", STACK_SIZE_BYTES, NULL, TASK_PRIORITY);

		TaskMonitorAdd(TASKINFO_RUNNING_AUTOTUNE, taskHandle);
		PIOS_WDG_RegisterFlag(PIOS_WDG_AUTOTUNE);
	}
	return 0;
}

MODULE_INITCALL(AutotuneInitialize, AutotuneStart)

static void at_new_gyro_data(UAVObjEvent * ev, void *ctx, void *obj, int len) {
	(void) ev; (void) ctx;

	static bool last_sample_unpushed = 0;

	GyrosData *g = obj;

	PIOS_Assert(len == sizeof(*g));

	if (last_sample_unpushed) {
		/* Last time we were unable to advance the write pointer.
		 * Try again, last chance! */
		if (circ_queue_advance_write(at_queue)) {
			at_points_spilled++;
		}
	}

	struct at_queued_data *q_item = circ_queue_cur_write_pos(at_queue);

	q_item->y[0] = g->x;
	q_item->y[1] = g->y;
	q_item->y[2] = g->z;

	q_item->dT = g->dT;
	q_item->ticks = g->SensorTicks;

	ActuatorDesiredData actuators;
	ActuatorDesiredGet(&actuators);

	q_item->u[0] = actuators.Roll;
	q_item->u[1] = actuators.Pitch;
	q_item->u[2] = actuators.Yaw;

	q_item->thrust = actuators.Thrust;

	if (circ_queue_advance_write(at_queue) != 0) {
		last_sample_unpushed = true;
	} else {
		last_sample_unpushed = false;
	}
}

static void UpdateSystemIdent(float gyroAccum[AUTOTUNE_CYCLE_LENGTH][3], float actuatorAccum[AUTOTUNE_CYCLE_LENGTH][3],
								float thrustAccum[AUTOTUNE_CYCLE_LENGTH], uint16_t numCycles, float dT_s, uint32_t predicts,
								uint32_t spills, float hover_thrust) {
	SystemIdentData system_ident;

	system_ident.Period = dT_s * 1000.0f;

	system_ident.NumAfPredicts = predicts;
	system_ident.NumSpilledPts = spills;
	system_ident.HoverThrust = hover_thrust;

	for (int i = 0; i < NUM_UAVO_ELEM; i++){
		system_ident.GyroAccumRoll[i] = gyroAccum[log_index + i][1] / numCycles;
		system_ident.GyroAccumPitch[i] = gyroAccum[log_index + i][2] / numCycles;
		system_ident.GyroAccumYaw[i] = gyroAccum[log_index + i][3] / numCycles;

		system_ident.ActuatorDesiredAccumRoll[i] = actuatorAccum[log_index + i][1] / numCycles;
		system_ident.ActuatorDesiredAccumPitch[i] = actuatorAccum[log_index + i][2] / numCycles;
		system_ident.ActuatorDesiredAccumYaw[i] = actuatorAccum[log_index + i][3] / numCycles;

		system_ident.ActuatorDesiredAccumThrust[i] = thrustAccum[log_index + i] / numCycles;
	}
	system_ident.AccumIndex = log_index;

	SystemIdentSet(&system_ident);

	log_index = (log_index + (NUM_UAVO_ELEM/2)) % AUTOTUNE_CYCLE_LENGTH;
}

static void UpdateStabilizationDesired(bool doingIdent) {
	StabilizationDesiredData stabDesired;
	StabilizationDesiredGet(&stabDesired);

	uint8_t rollMax, pitchMax;

	float manualRate[STABILIZATIONSETTINGS_MANUALRATE_NUMELEM];

	StabilizationSettingsRollMaxGet(&rollMax);
	StabilizationSettingsPitchMaxGet(&pitchMax);
	StabilizationSettingsManualRateGet(manualRate);

	SystemSettingsAirframeTypeOptions airframe_type;
	SystemSettingsAirframeTypeGet(&airframe_type);

	ManualControlCommandData manual_control_command;
	ManualControlCommandGet(&manual_control_command);

	stabDesired.Roll = manual_control_command.Roll * rollMax;
	stabDesired.Pitch = manual_control_command.Pitch * pitchMax;
	stabDesired.Yaw = manual_control_command.Yaw * manualRate[STABILIZATIONSETTINGS_MANUALRATE_YAW];

	if (doingIdent) {
		stabDesired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_ROLL]  = STABILIZATIONDESIRED_STABILIZATIONMODE_SYSTEMIDENT;
		stabDesired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_PITCH] = STABILIZATIONDESIRED_STABILIZATIONMODE_SYSTEMIDENT;
		stabDesired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_YAW] = STABILIZATIONDESIRED_STABILIZATIONMODE_SYSTEMIDENT;
	} else {
		stabDesired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_ROLL]  = STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
		stabDesired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_PITCH] = STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
		stabDesired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_YAW] = STABILIZATIONDESIRED_STABILIZATIONMODE_RATE;
	}

	stabDesired.Thrust = (airframe_type == SYSTEMSETTINGS_AIRFRAMETYPE_HELICP) ? manual_control_command.Collective : manual_control_command.Throttle;
	StabilizationDesiredSet(&stabDesired);
}

#define MAX_PTS_PER_CYCLE 4

/**
 * Module thread, should not return.
 */
static void AutotuneTask(void *parameters)
{
	enum AUTOTUNE_STATE state = AT_INIT;

	uint32_t last_update_time = PIOS_Thread_Systime();

	float gyroAccum[AUTOTUNE_CYCLE_LENGTH][3] = {0};
	float actuatorAccum[AUTOTUNE_CYCLE_LENGTH][3] = {0};
	float thrustAccum[AUTOTUNE_CYCLE_LENGTH] = {0};
	uint16_t numCycles = 0;

	af_init();

	const uint32_t YIELD_MS = 2;

	GyrosConnectCallback(at_new_gyro_data);

	bool save_needed = false;

	while(1) {
		PIOS_WDG_UpdateFlag(PIOS_WDG_AUTOTUNE);

		uint32_t diff_time;

		const uint32_t PREPARE_TIME = 2000;
		const uint32_t MEASURE_TIME = 60000;

		static uint32_t update_counter = 0;

		bool doing_ident = false;
		bool can_sleep = true;

		FlightStatusData flightStatus;
		FlightStatusGet(&flightStatus);

		if (save_needed) {
			if (flightStatus.Armed == FLIGHTSTATUS_ARMED_DISARMED) {
				// Save the settings locally.
				UAVObjSave(SystemIdentHandle(), 0);
				state = AT_INIT;

				save_needed = false;
			}

		}

		// Only allow this module to run when autotuning
		if (flightStatus.FlightMode != FLIGHTSTATUS_FLIGHTMODE_AUTOTUNE) {
			state = AT_INIT;
			PIOS_Thread_Sleep(50);
			continue;
		}

		switch(state) {
			case AT_INIT:
				// Reset save status; only save if this tune
				// completes.
				save_needed = false;

				last_update_time = PIOS_Thread_Systime();

				// Only start when armed and flying
				if (flightStatus.Armed == FLIGHTSTATUS_ARMED_ARMED) {

					af_init();

					UpdateSystemIdent(gyroAccum, actuatorAccum, thrustAccum, 1, 0.0f, 0, 0, 0.0f);

					state = AT_START;

				}

				break;

			case AT_START:

				diff_time = PIOS_Thread_Systime() - last_update_time;

				// Spend the first block of time in normal rate mode to get stabilized
				if (diff_time > PREPARE_TIME) {

					/* Drain the queue of all current data */
					while (circ_queue_read_pos(at_queue)) {
						circ_queue_read_completed(at_queue);
					}

					/* And reset the point spill counter */

					update_counter = 0;
					at_points_spilled = 0;

					thrust_accumulator = 0;
					numCycles = 1;

					state = AT_RUN;
					last_update_time = PIOS_Thread_Systime();
				}


				break;

			case AT_RUN:

				diff_time = PIOS_Thread_Systime() - last_update_time;

				doing_ident = true;
				can_sleep = false;

				for (int i=0; i<MAX_PTS_PER_CYCLE; i++) {
					struct at_queued_data *pt;

					/* Grab an autotune point */
					pt = circ_queue_read_pos(at_queue);

					if (!pt) {
						/* We've drained the buffer
						 * fully.  Yay! */
						can_sleep = true;
						break;
					}

					if (!(pt->ticks % AUTOTUNE_CYCLE_LENGTH)) {
						numCycles++;
					}

					/* This is for the first point, but
					 * also if we have extended drops */
					float dT_s = 0.010f;
					if ((pt->dT > 0.0f) && (pt->dT < 0.010f)) {
						dT_s = pt->dT;
					}

					af_predict(gyroAccum, actuatorAccum, thrustAccum, pt->u, pt->y, pt->ticks, pt->thrust);

					//This will work up to 8kHz with an 89% thrust position before overflow
					thrust_accumulator += 10000 * pt->thrust;

					// Update uavo every 256 cycles to avoid
					// telemetry spam
					if (!((update_counter++) & 0xff)) {
						float hover_thrust = ((float)(thrust_accumulator/update_counter))/10000.0f;
						UpdateSystemIdent(gyroAccum, actuatorAccum, thrustAccum, numCycles, dT_s, update_counter, at_points_spilled, hover_thrust);
					}

					/* Free the buffer containing an AT point */
					circ_queue_read_completed(at_queue);
				}

				if (diff_time > MEASURE_TIME) { // Move on to next state
					state = AT_FINISHED;
					last_update_time = PIOS_Thread_Systime();
				}

				break;

			case AT_FINISHED: ;

				// Wait until disarmed and landed before saving the settings

				float hover_thrust = ((float)(thrust_accumulator/update_counter))/10000.0f;
				UpdateSystemIdent(gyroAccum, actuatorAccum, thrustAccum, numCycles, 0, update_counter, at_points_spilled, hover_thrust);

				save_needed = true;
				state = AT_WAITING;

				break;

			case AT_WAITING:
				if (!((update_counter++) % 50)) {
					float hover_thrust = ((float)(thrust_accumulator/update_counter))/10000.0f;
					UpdateSystemIdent(gyroAccum, actuatorAccum, thrustAccum, numCycles, 0, update_counter, at_points_spilled, hover_thrust);
				}
				break;

			default:
				// Set an alarm or some shit like that
				break;
		}

		// Update based on manual controls
		UpdateStabilizationDesired(doing_ident);

		if (can_sleep) {
			PIOS_Thread_Sleep(YIELD_MS);
		}
	}
}

/**
 * Prediction step for EKF on control inputs to quad that
 * learns the system properties
 * @param X the current state estimate which is updated in place
 * @param P the current covariance matrix, updated in place
 * @param[in] the current control inputs (roll, pitch, yaw)
 * @param[in] the gyro measurements
 */
__attribute__((always_inline)) static inline void af_predict(float gyroAccum[AUTOTUNE_CYCLE_LENGTH][3], float actuatorAccum[AUTOTUNE_CYCLE_LENGTH][3],
																float thrustAccum[AUTOTUNE_CYCLE_LENGTH], const float u_in[3], const float gyro[3],
																const uint32_t ticks, const float t_in)
{
	uint16_t index = ticks % AUTOTUNE_CYCLE_LENGTH;

	for (int i = 0; i < 3; i++) {
		gyroAccum[index][i] += gyro[i];
		actuatorAccum[index][i] += u_in[i];
	}
	thrustAccum[index] += t_in;
}

/**
 * Initialize the state variable and covariance matrix
 * for the system identification EKF
 */
static void af_init()
{

}

/**
 * @}
 * @}
 */
