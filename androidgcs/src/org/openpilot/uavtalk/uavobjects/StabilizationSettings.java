/**
 ******************************************************************************
 *
 * @file       uavobjecttemplate.cpp
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Template for an uavobject in java
 *             This is a autogenerated file!! Do not modify and expect a result.
 *             PID settings used by the Stabilization module to combine the @ref AttitudeActual and @ref AttitudeDesired to compute @ref ActuatorDesired
 *
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

package org.openpilot.uavtalk.uavobjects;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.ListIterator;

import org.openpilot.uavtalk.UAVObjectManager;
import org.openpilot.uavtalk.UAVObject;
import org.openpilot.uavtalk.UAVDataObject;
import org.openpilot.uavtalk.UAVObjectField;

/**
PID settings used by the Stabilization module to combine the @ref AttitudeActual and @ref AttitudeDesired to compute @ref ActuatorDesired

generated from stabilizationsettings.xml
 **/
public class StabilizationSettings extends UAVDataObject {

	public StabilizationSettings() {
		super(OBJID, ISSINGLEINST, ISSETTINGS, NAME);
		
		List<UAVObjectField> fields = new ArrayList<UAVObjectField>();
		

		List<String> ManualRateElemNames = new ArrayList<String>();
		ManualRateElemNames.add("Roll");
		ManualRateElemNames.add("Pitch");
		ManualRateElemNames.add("Yaw");
		fields.add( new UAVObjectField("ManualRate", "degrees/sec", UAVObjectField.FieldType.FLOAT32, ManualRateElemNames, null) );

		List<String> MaximumRateElemNames = new ArrayList<String>();
		MaximumRateElemNames.add("Roll");
		MaximumRateElemNames.add("Pitch");
		MaximumRateElemNames.add("Yaw");
		fields.add( new UAVObjectField("MaximumRate", "degrees/sec", UAVObjectField.FieldType.FLOAT32, MaximumRateElemNames, null) );

		List<String> RollRatePIDElemNames = new ArrayList<String>();
		RollRatePIDElemNames.add("Kp");
		RollRatePIDElemNames.add("Ki");
		RollRatePIDElemNames.add("Kd");
		RollRatePIDElemNames.add("ILimit");
		fields.add( new UAVObjectField("RollRatePID", "", UAVObjectField.FieldType.FLOAT32, RollRatePIDElemNames, null) );

		List<String> PitchRatePIDElemNames = new ArrayList<String>();
		PitchRatePIDElemNames.add("Kp");
		PitchRatePIDElemNames.add("Ki");
		PitchRatePIDElemNames.add("Kd");
		PitchRatePIDElemNames.add("ILimit");
		fields.add( new UAVObjectField("PitchRatePID", "", UAVObjectField.FieldType.FLOAT32, PitchRatePIDElemNames, null) );

		List<String> YawRatePIDElemNames = new ArrayList<String>();
		YawRatePIDElemNames.add("Kp");
		YawRatePIDElemNames.add("Ki");
		YawRatePIDElemNames.add("Kd");
		YawRatePIDElemNames.add("ILimit");
		fields.add( new UAVObjectField("YawRatePID", "", UAVObjectField.FieldType.FLOAT32, YawRatePIDElemNames, null) );

		List<String> RollPIElemNames = new ArrayList<String>();
		RollPIElemNames.add("Kp");
		RollPIElemNames.add("Ki");
		RollPIElemNames.add("ILimit");
		fields.add( new UAVObjectField("RollPI", "", UAVObjectField.FieldType.FLOAT32, RollPIElemNames, null) );

		List<String> PitchPIElemNames = new ArrayList<String>();
		PitchPIElemNames.add("Kp");
		PitchPIElemNames.add("Ki");
		PitchPIElemNames.add("ILimit");
		fields.add( new UAVObjectField("PitchPI", "", UAVObjectField.FieldType.FLOAT32, PitchPIElemNames, null) );

		List<String> YawPIElemNames = new ArrayList<String>();
		YawPIElemNames.add("Kp");
		YawPIElemNames.add("Ki");
		YawPIElemNames.add("ILimit");
		fields.add( new UAVObjectField("YawPI", "", UAVObjectField.FieldType.FLOAT32, YawPIElemNames, null) );

		List<String> VbarSensitivityElemNames = new ArrayList<String>();
		VbarSensitivityElemNames.add("Roll");
		VbarSensitivityElemNames.add("Pitch");
		VbarSensitivityElemNames.add("Yaw");
		fields.add( new UAVObjectField("VbarSensitivity", "frac", UAVObjectField.FieldType.FLOAT32, VbarSensitivityElemNames, null) );

		List<String> VbarRollPIElemNames = new ArrayList<String>();
		VbarRollPIElemNames.add("Kp");
		VbarRollPIElemNames.add("Ki");
		fields.add( new UAVObjectField("VbarRollPI", "1/(deg/s)", UAVObjectField.FieldType.FLOAT32, VbarRollPIElemNames, null) );

		List<String> VbarPitchPIElemNames = new ArrayList<String>();
		VbarPitchPIElemNames.add("Kp");
		VbarPitchPIElemNames.add("Ki");
		fields.add( new UAVObjectField("VbarPitchPI", "1/(deg/s)", UAVObjectField.FieldType.FLOAT32, VbarPitchPIElemNames, null) );

		List<String> VbarYawPIElemNames = new ArrayList<String>();
		VbarYawPIElemNames.add("Kp");
		VbarYawPIElemNames.add("Ki");
		fields.add( new UAVObjectField("VbarYawPI", "1/(deg/s)", UAVObjectField.FieldType.FLOAT32, VbarYawPIElemNames, null) );

		List<String> VbarTauElemNames = new ArrayList<String>();
		VbarTauElemNames.add("0");
		fields.add( new UAVObjectField("VbarTau", "sec", UAVObjectField.FieldType.FLOAT32, VbarTauElemNames, null) );

		List<String> GyroTauElemNames = new ArrayList<String>();
		GyroTauElemNames.add("0");
		fields.add( new UAVObjectField("GyroTau", "", UAVObjectField.FieldType.FLOAT32, GyroTauElemNames, null) );

		List<String> WeakLevelingKpElemNames = new ArrayList<String>();
		WeakLevelingKpElemNames.add("0");
		fields.add( new UAVObjectField("WeakLevelingKp", "(deg/s)/deg", UAVObjectField.FieldType.FLOAT32, WeakLevelingKpElemNames, null) );

		List<String> RollMaxElemNames = new ArrayList<String>();
		RollMaxElemNames.add("0");
		fields.add( new UAVObjectField("RollMax", "degrees", UAVObjectField.FieldType.UINT8, RollMaxElemNames, null) );

		List<String> PitchMaxElemNames = new ArrayList<String>();
		PitchMaxElemNames.add("0");
		fields.add( new UAVObjectField("PitchMax", "degrees", UAVObjectField.FieldType.UINT8, PitchMaxElemNames, null) );

		List<String> YawMaxElemNames = new ArrayList<String>();
		YawMaxElemNames.add("0");
		fields.add( new UAVObjectField("YawMax", "degrees", UAVObjectField.FieldType.UINT8, YawMaxElemNames, null) );

		List<String> VbarGyroSuppressElemNames = new ArrayList<String>();
		VbarGyroSuppressElemNames.add("0");
		fields.add( new UAVObjectField("VbarGyroSuppress", "%", UAVObjectField.FieldType.INT8, VbarGyroSuppressElemNames, null) );

		List<String> VbarPiroCompElemNames = new ArrayList<String>();
		VbarPiroCompElemNames.add("0");
		List<String> VbarPiroCompEnumOptions = new ArrayList<String>();
		VbarPiroCompEnumOptions.add("FALSE");
		VbarPiroCompEnumOptions.add("TRUE");
		fields.add( new UAVObjectField("VbarPiroComp", "", UAVObjectField.FieldType.ENUM, VbarPiroCompElemNames, VbarPiroCompEnumOptions) );

		List<String> VbarMaxAngleElemNames = new ArrayList<String>();
		VbarMaxAngleElemNames.add("0");
		fields.add( new UAVObjectField("VbarMaxAngle", "deg", UAVObjectField.FieldType.UINT8, VbarMaxAngleElemNames, null) );

		List<String> MaxAxisLockElemNames = new ArrayList<String>();
		MaxAxisLockElemNames.add("0");
		fields.add( new UAVObjectField("MaxAxisLock", "deg", UAVObjectField.FieldType.UINT8, MaxAxisLockElemNames, null) );

		List<String> MaxAxisLockRateElemNames = new ArrayList<String>();
		MaxAxisLockRateElemNames.add("0");
		fields.add( new UAVObjectField("MaxAxisLockRate", "deg/s", UAVObjectField.FieldType.UINT8, MaxAxisLockRateElemNames, null) );

		List<String> MaxWeakLevelingRateElemNames = new ArrayList<String>();
		MaxWeakLevelingRateElemNames.add("0");
		fields.add( new UAVObjectField("MaxWeakLevelingRate", "deg/s", UAVObjectField.FieldType.UINT8, MaxWeakLevelingRateElemNames, null) );

		List<String> LowThrottleZeroIntegralElemNames = new ArrayList<String>();
		LowThrottleZeroIntegralElemNames.add("0");
		List<String> LowThrottleZeroIntegralEnumOptions = new ArrayList<String>();
		LowThrottleZeroIntegralEnumOptions.add("FALSE");
		LowThrottleZeroIntegralEnumOptions.add("TRUE");
		fields.add( new UAVObjectField("LowThrottleZeroIntegral", "", UAVObjectField.FieldType.ENUM, LowThrottleZeroIntegralElemNames, LowThrottleZeroIntegralEnumOptions) );


		// Compute the number of bytes for this object
		int numBytes = 0;
		ListIterator<UAVObjectField> li = fields.listIterator();
		while(li.hasNext()) {
			numBytes += li.next().getNumBytes();
		}
		NUMBYTES = numBytes;

		// Initialize object
		initializeFields(fields, ByteBuffer.allocate(NUMBYTES), NUMBYTES);
		// Set the default field values
		setDefaultFieldValues();
		// Set the object description
		setDescription(DESCRIPTION);
	}

	/**
	 * Create a Metadata object filled with default values for this object
	 * @return Metadata object with default values
	 */
	public Metadata getDefaultMetadata() {
		UAVObject.Metadata metadata = new UAVObject.Metadata();
    	metadata.flags =
		    UAVObject.Metadata.AccessModeNum(UAVObject.AccessMode.ACCESS_READWRITE) << UAVOBJ_ACCESS_SHIFT |
		    UAVObject.Metadata.AccessModeNum(UAVObject.AccessMode.ACCESS_READWRITE) << UAVOBJ_GCS_ACCESS_SHIFT |
		    1 << UAVOBJ_TELEMETRY_ACKED_SHIFT |
		    1 << UAVOBJ_GCS_TELEMETRY_ACKED_SHIFT |
		    UAVObject.Metadata.UpdateModeNum(UAVObject.UpdateMode.UPDATEMODE_ONCHANGE) << UAVOBJ_TELEMETRY_UPDATE_MODE_SHIFT |
		    UAVObject.Metadata.UpdateModeNum(UAVObject.UpdateMode.UPDATEMODE_ONCHANGE) << UAVOBJ_GCS_TELEMETRY_UPDATE_MODE_SHIFT;
    	metadata.flightTelemetryUpdatePeriod = 0;
    	metadata.gcsTelemetryUpdatePeriod = 0;
    	metadata.loggingUpdatePeriod = 0;
 
		return metadata;
	}

	/**
	 * Initialize object fields with the default values.
	 * If a default value is not specified the object fields
	 * will be initialized to zero.
	 */
	public void setDefaultFieldValues()
	{
		getField("ManualRate").setValue(150,0);
		getField("ManualRate").setValue(150,1);
		getField("ManualRate").setValue(150,2);
		getField("MaximumRate").setValue(300,0);
		getField("MaximumRate").setValue(300,1);
		getField("MaximumRate").setValue(300,2);
		getField("RollRatePID").setValue(0.002,0);
		getField("RollRatePID").setValue(0,1);
		getField("RollRatePID").setValue(0,2);
		getField("RollRatePID").setValue(0.3,3);
		getField("PitchRatePID").setValue(0.002,0);
		getField("PitchRatePID").setValue(0,1);
		getField("PitchRatePID").setValue(0,2);
		getField("PitchRatePID").setValue(0.3,3);
		getField("YawRatePID").setValue(0.0035,0);
		getField("YawRatePID").setValue(0.0035,1);
		getField("YawRatePID").setValue(0,2);
		getField("YawRatePID").setValue(0.3,3);
		getField("RollPI").setValue(2,0);
		getField("RollPI").setValue(0,1);
		getField("RollPI").setValue(50,2);
		getField("PitchPI").setValue(2,0);
		getField("PitchPI").setValue(0,1);
		getField("PitchPI").setValue(50,2);
		getField("YawPI").setValue(2,0);
		getField("YawPI").setValue(0,1);
		getField("YawPI").setValue(50,2);
		getField("VbarSensitivity").setValue(0.5,0);
		getField("VbarSensitivity").setValue(0.5,1);
		getField("VbarSensitivity").setValue(0.5,2);
		getField("VbarRollPI").setValue(0.005,0);
		getField("VbarRollPI").setValue(0.002,1);
		getField("VbarPitchPI").setValue(0.005,0);
		getField("VbarPitchPI").setValue(0.002,1);
		getField("VbarYawPI").setValue(0.005,0);
		getField("VbarYawPI").setValue(0.002,1);
		getField("VbarTau").setValue(0.5);
		getField("GyroTau").setValue(0.005);
		getField("WeakLevelingKp").setValue(0.1);
		getField("RollMax").setValue(55);
		getField("PitchMax").setValue(55);
		getField("YawMax").setValue(35);
		getField("VbarGyroSuppress").setValue(30);
		getField("VbarPiroComp").setValue("FALSE");
		getField("VbarMaxAngle").setValue(10);
		getField("MaxAxisLock").setValue(15);
		getField("MaxAxisLockRate").setValue(2);
		getField("MaxWeakLevelingRate").setValue(5);
		getField("LowThrottleZeroIntegral").setValue("TRUE");

	}

	/**
	 * Create a clone of this object, a new instance ID must be specified.
	 * Do not use this function directly to create new instances, the
	 * UAVObjectManager should be used instead.
	 */
	public UAVDataObject clone(int instID) {
		// TODO: Need to get specific instance to clone
		try {
			StabilizationSettings obj = new StabilizationSettings();
			obj.initialize(instID, this.getMetaObject());
			return obj;
		} catch  (Exception e) {
			return null;
		}
	}

	/**
	 * Static function to retrieve an instance of the object.
	 */
	public StabilizationSettings GetInstance(UAVObjectManager objMngr, int instID)
	{
	    return (StabilizationSettings)(objMngr.getObject(StabilizationSettings.OBJID, instID));
	}

	// Constants
	protected static final int OBJID = 0xBBC337D4;
	protected static final String NAME = "StabilizationSettings";
	protected static String DESCRIPTION = "PID settings used by the Stabilization module to combine the @ref AttitudeActual and @ref AttitudeDesired to compute @ref ActuatorDesired";
	protected static final boolean ISSINGLEINST = 1 == 1;
	protected static final boolean ISSETTINGS = 1 == 1;
	protected static int NUMBYTES = 0;


}