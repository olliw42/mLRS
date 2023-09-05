//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// LINK TYPES
//*******************************************************

#include "link_types.h"


// for debug purposes

const char* connectstate_str[] = { "L", "S", "C" };

#ifdef DEVICE_IS_TRANSMITTER
const char* linkstate_str[] = { "i", "t", "tw", "r", "rw", "d" };
const char* rxstatus_str[] = { "n", "i", "v" };
#else
const char* linkstate_str[] = { "r", "rw", "t", "tw" };
const char* rxstatus_str[] = { "n", "i", "c", "v" };
#endif


