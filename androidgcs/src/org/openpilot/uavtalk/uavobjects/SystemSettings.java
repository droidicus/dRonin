/**
 ******************************************************************************
 *
 * @file       uavobjecttemplate.cpp
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Template for an uavobject in java
 *             This is a autogenerated file!! Do not modify and expect a result.
 *             Select airframe type.  Currently used by @ref ActuatorModule to choose mixing from @ref ActuatorDesired to @ref ActuatorCommand
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
Select airframe type.  Currently used by @ref ActuatorModule to choose mixing from @ref ActuatorDesired to @ref ActuatorCommand

generated from systemsettings.xml
 **/
public class SystemSettings extends UAVDataObject {

	public SystemSettings() {
		super(OBJID, ISSINGLEINST, ISSETTINGS, NAME);
		
		List<UAVObjectField> fields = new ArrayList<UAVObjectField>();
		

		List<String> GUIConfigDataElemNames = new ArrayList<String>();
		GUIConfigDataElemNames.add("0");
		GUIConfigDataElemNames.add("1");
		GUIConfigDataElemNames.add("2");
		GUIConfigDataElemNames.add("3");
		fields.add( new UAVObjectField("GUIConfigData", "bits", UAVObjectField.FieldType.UINT32, GUIConfigDataElemNames, null) );

		List<String> AirframeTypeElemNames = new ArrayList<String>();
		AirframeTypeElemNames.add("0");
		List<String> AirframeTypeEnumOptions = new ArrayList<String>();
		AirframeTypeEnumOptions.add("FixedWing");
		AirframeTypeEnumOptions.add("FixedWingElevon");
		AirframeTypeEnumOptions.add("FixedWingVtail");
		AirframeTypeEnumOptions.add("VTOL");
		AirframeTypeEnumOptions.add("HeliCP");
		AirframeTypeEnumOptions.add("QuadX");
		AirframeTypeEnumOptions.add("QuadP");
		AirframeTypeEnumOptions.add("Hexa");
		AirframeTypeEnumOptions.add("Octo");
		AirframeTypeEnumOptions.add("Custom");
		AirframeTypeEnumOptions.add("HexaX");
		AirframeTypeEnumOptions.add("OctoV");
		AirframeTypeEnumOptions.add("OctoCoaxP");
		AirframeTypeEnumOptions.add("OctoCoaxX");
		AirframeTypeEnumOptions.add("HexaCoax");
		AirframeTypeEnumOptions.add("Tri");
		AirframeTypeEnumOptions.add("GroundVehicleCar");
		AirframeTypeEnumOptions.add("GroundVehicleDifferential");
		AirframeTypeEnumOptions.add("GroundVehicleMotorcycle");
		fields.add( new UAVObjectField("AirframeType", "", UAVObjectField.FieldType.ENUM, AirframeTypeElemNames, AirframeTypeEnumOptions) );


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
		getField("GUIConfigData").setValue(0,0);
		getField("GUIConfigData").setValue(0,1);
		getField("GUIConfigData").setValue(0,2);
		getField("GUIConfigData").setValue(0,3);
		getField("AirframeType").setValue("QuadX");

	}

	/**
	 * Create a clone of this object, a new instance ID must be specified.
	 * Do not use this function directly to create new instances, the
	 * UAVObjectManager should be used instead.
	 */
	public UAVDataObject clone(int instID) {
		// TODO: Need to get specific instance to clone
		try {
			SystemSettings obj = new SystemSettings();
			obj.initialize(instID, this.getMetaObject());
			return obj;
		} catch  (Exception e) {
			return null;
		}
	}

	/**
	 * Static function to retrieve an instance of the object.
	 */
	public SystemSettings GetInstance(UAVObjectManager objMngr, int instID)
	{
	    return (SystemSettings)(objMngr.getObject(SystemSettings.OBJID, instID));
	}

	// Constants
	protected static final int OBJID = 0xC72A326E;
	protected static final String NAME = "SystemSettings";
	protected static String DESCRIPTION = "Select airframe type.  Currently used by @ref ActuatorModule to choose mixing from @ref ActuatorDesired to @ref ActuatorCommand";
	protected static final boolean ISSINGLEINST = 1 == 1;
	protected static final boolean ISSETTINGS = 1 == 1;
	protected static int NUMBYTES = 0;


}