/*
* Copyright (C) 2013, Osnabrück University
* Copyright (C) 2017, Ing.-Buero Dr. Michael Lehning, Hildesheim
* Copyright (C) 2017, SICK AG, Waldkirch
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Osnabrück University nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*     * Neither the name of SICK AG nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission
*     * Neither the name of Ing.-Buero Dr. Michael Lehning nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
*  Last modified: 12th Dec 2017
*
*      Authors:
*         Michael Lehning <michael.lehning@lehning.de>
*         Jochen Sprickerhof <jochen@sprickerhof.de>
*         Martin Günther <mguenthe@uos.de>
*
* Based on the TiM communication example by SICK AG.
*
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sick_scan/sick_generic_laser.h"


#ifdef _MSC_VER
#include "sick_scan/rosconsole_simu.hpp"
#endif

#define MAX_NAME_LEN (1024)

#define SICK_GENERIC_MAJOR_VER "001"
#define SICK_GENERIC_MINOR_VER "000"
#define SICK_GENERIC_PATCH_LEVEL "000"


int main(int argc, char **argv)
{
	char nameId[] = "__name:=";
	char nameVal[MAX_NAME_LEN] = { 0 };
	char **argv_tmp; // argv_tmp[0][0] argv_tmp[0] identisch ist zu (*argv_tmp)
	int argc_tmp;
	std::string scannerName = "????";


	argc_tmp = argc;
	argv_tmp = argv;

	const int MAX_STR_LEN = 1024;
	char nameTagVal[MAX_STR_LEN] = { 0 };
	char logTagVal[MAX_STR_LEN] = { 0 };
	char internalDebugTagVal[MAX_STR_LEN] = { 0 };

	if (argc == 1) // just for testing without calling by roslaunch
	{
		strcpy(nameTagVal, "__name:=sick_tim_5xx");
		// strcpy(nameTagVal, "__name:=sick_mrs_1xxx"); // dann IP-Adresse auf ...2 stellen
		strcpy(logTagVal, "__log:=/tmp/tmp.log");
		strcpy(internalDebugTagVal, "__internalDebug:=1");
		argc_tmp = 4;
		argv_tmp = (char **)malloc(sizeof(char *) * argc_tmp);

		argv_tmp[0] = argv[0];
		argv_tmp[1] = nameTagVal;
		argv_tmp[2] = logTagVal;
		argv_tmp[3] = internalDebugTagVal;

	}
  ROS_INFO("sick_generic_call V. %s.%s.%s", SICK_GENERIC_MAJOR_VER, SICK_GENERIC_MINOR_VER, SICK_GENERIC_PATCH_LEVEL);
	for (int i = 0; i < argc_tmp; i++)
	{
		if (strstr(argv_tmp[i], nameId) == argv_tmp[i])
		{
			strcpy(nameVal, argv_tmp[i] + strlen(nameId));
			scannerName = nameVal;
		}
		ROS_INFO("Program arguments: %s", argv_tmp[i]);
	}


	int result = mainGenericLaser(argc_tmp, argv_tmp, scannerName);
	return result;

}
