// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//
// NMEA parser, adapted by Michael Smith from TinyGPS v9:
//
// TinyGPS - a small GPS library for Arduino providing basic NMEA parsing
// Copyright (C) 2008-9 Mikal Hart
// All rights reserved.
//
//

/// @file	AP_GPS_NMEA.h
/// @brief	NMEA protocol parser
///
/// This is a lightweight NMEA parser, derived originally from the
/// TinyGPS parser by Mikal Hart.  It is frugal in its use of memory
/// and tries to avoid unnecessary arithmetic.
///
/// The parser handles GPGGA, GPRMC and GPVTG messages, and attempts to be
/// robust in the face of occasional corruption in the input stream.  It
/// makes a basic effort to configure GPS' that are likely to be connected in
/// NMEA mode (SiRF, MediaTek and ublox) to emit the correct message
/// stream, but does not validate that the correct stream is being received.
/// In particular, a unit emitting just GPRMC will show as having a fix
/// even though no altitude data is being received.
///
/// GPVTG data is parsed, but as the message may not contain the the
/// qualifier field (this is common with e.g. older SiRF units) it is
/// not considered a source of fix-valid information.
///
#pragma once

#include "AP_GPS_NMEA.h"

/// NMEA parser
///
class AP_GPS_MAVLINK : public AP_GPS_NMEA
{

public:
	AP_GPS_MAVLINK(AP_GPS &_gps, AP_GPS::GPS_State &_state);

    /// Checks the serial receive buffer for characters,
    /// attempts to parse NMEA data and updates internal state
    /// accordingly.
    bool        read();

    // Not for DGPS, but to actually inject the aircraft's gps position via Mavlink
    void inject_data(uint8_t *data, uint8_t len);

private:
    bool _new_data;
};
