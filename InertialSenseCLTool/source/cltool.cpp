/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "cltool.h"

cmd_options_t g_commandLineOptions;
serial_port_t g_serialPort;
cInertialSenseDisplay g_inertialSenseDisplay;
bool g_ctrlCPressed;

#if defined(_WIN32)

#include <Windows.h>

static bool ctrlHandler(DWORD fdwCtrlType)
{
	switch (fdwCtrlType)
	{
	case CTRL_C_EVENT:
	case CTRL_CLOSE_EVENT:
	case CTRL_BREAK_EVENT:
	case CTRL_LOGOFF_EVENT:
	case CTRL_SHUTDOWN_EVENT:
		g_ctrlCPressed = true;
		return true;
	default:
		return false;
  }
}

#else

#include <signal.h>

static void signalFunction(int sig)
{
	g_ctrlCPressed = true; // set flag
}

#endif

int cltool_serialPortSendComManager(CMHANDLE cmHandle, int pHandle, buffer_t* bufferToSend)
{
	(void)cmHandle;
	(void)pHandle;
	return serialPortWrite(&g_serialPort, bufferToSend->buf, bufferToSend->size);
}

void cltool_setupLogger(InertialSense& inertialSenseInterface)
{
	// Enable logging in continuous background mode
	inertialSenseInterface.SetLoggerEnabled
	(
		g_commandLineOptions.enableLogging, // enable logger
		g_commandLineOptions.logPath, // path to log to, if empty defaults to DEFAULT_LOGS_DIRECTORY
		g_commandLineOptions.logSolution, // solution logging options
		g_commandLineOptions.maxLogSpaceMB, // max space in mb to use, 0 for unlimited - only MAX_PERCENT_OF_FREE_SPACE_TO_USE_FOR_IS_LOGS% of free space will ever be allocated
		g_commandLineOptions.maxLogFileSize, // each log file will be no larger than this in bytes
		g_commandLineOptions.maxLogMemory, // logger will try and keep under this amount of memory
		g_commandLineOptions.useLogTimestampSubFolder // whether to place log files in a new sub-folder with the current timestamp as the folder name
	);

	// Call these elsewhere as needed
// 	inertialSenseInterface.EnableLogger(false);	// Enable/disable during runtime
// 	inertialSenseInterface.CloseLogger();		// Stop logging and save remaining data to file
}

static bool startsWith(const char* str, const char* pre)
{
	size_t lenpre = strlen(pre), lenstr = strlen(str);
	return lenstr < lenpre ? false : strncmp(pre, str, lenpre) == 0;
}

bool cltool_parseCommandLine(int argc, char* argv[])
{
	// set defaults
	g_commandLineOptions.displayMode = cInertialSenseDisplay::DMODE_PRETTY;
	g_commandLineOptions.logPath = DEFAULT_LOGS_DIRECTORY;
	g_commandLineOptions.logSolution = SLOG_W_INS2;
	g_commandLineOptions.maxLogFileSize = 1024 * 1024 * 5;
	g_commandLineOptions.maxLogSpaceMB = 1024.0f;
	g_commandLineOptions.maxLogMemory = 131072;
	g_commandLineOptions.replaySpeed = 1.0;
	g_commandLineOptions.enableLogging = true;

	// parse command line
	for (int i = 1; i < argc; i++)
	{
		const char* a = argv[i];
		if (!strncmp(a, "-h", 2))
		{
			cltool_outputUsage();
			return false;
		}
		else if (startsWith(a, "-c="))
		{
			g_commandLineOptions.comPort = &a[3];
		}
		else if (startsWith(a, "-b="))
		{
			g_commandLineOptions.bootloaderFileName = &a[3];
		}
		else if (!strncmp(a, "-sINS1", 6))
		{
			g_commandLineOptions.streamINS1 = true;
		}
		else if (!strncmp(a, "-sINS2", 6))
		{
			g_commandLineOptions.streamINS2 = true;
		}
		else if (!strncmp(a, "-sDualIMU", 9))
		{
			g_commandLineOptions.streamDualIMU = true;
		}
		else if (!strncmp(a, "-sIMU1", 6))
		{
			g_commandLineOptions.streamIMU1 = true;
		}
		else if (!strncmp(a, "-sIMU2", 6))
		{
			g_commandLineOptions.streamIMU2 = true;
		}
		else if (!strncmp(a, "-sGPS", 5))
		{
			g_commandLineOptions.streamGPS = true;
		}
		else if (!strncmp(a, "-sMag1", 6))
		{
			g_commandLineOptions.streamMag1 = true;
		}
		else if (!strncmp(a, "-sBaro", 6))
		{
			g_commandLineOptions.streamBaro = true;
		}
		else if (!strncmp(a, "-sSol", 5))
		{
			g_commandLineOptions.streamSol = true;
			g_commandLineOptions.logSolution = SLOG_W_INS2;
		}
		else if (!strncmp(a, "-sSensors", 9))
		{
			g_commandLineOptions.streamSysSensors = true;
		}
		else if (!strncmp(a, "-sDThetaVel", 11))
		{
			g_commandLineOptions.streamDThetaVel = true;
		}
		else if (!strncmp(a, "-scroll", 7))
		{
			g_commandLineOptions.displayMode = cInertialSenseDisplay::DMODE_SCROLL;
		}
		else if (!strncmp(a, "-stats", 6))
		{
			g_commandLineOptions.displayMode = cInertialSenseDisplay::DMODE_STATS;
		}
		else if (startsWith(a, "-lms="))
		{
			g_commandLineOptions.maxLogSpaceMB = (float)atof(&a[5]);
		}
		else if (startsWith(a, "-lmf="))
		{
			g_commandLineOptions.maxLogFileSize = (uint32_t)strtoul(&a[5], NULL, 10);
		}
		else if (startsWith(a, "-lmm="))
		{
			g_commandLineOptions.maxLogMemory = (uint32_t)strtoul(&a[5], NULL, 10);
		}
		else if (startsWith(a, "-lts="))
		{
			g_commandLineOptions.useLogTimestampSubFolder = (a[5] == '1' || startsWith(&a[5], "true"));
		}
		else if (startsWith(a, "-lp="))
		{
			g_commandLineOptions.logPath = &a[4];
		}
		else if (!strncmp(a, "-loff", 5))
		{
			g_commandLineOptions.enableLogging = false;
		}
		else if (startsWith(a, "-rs="))
		{
			g_commandLineOptions.replayDataLog = true;
			g_commandLineOptions.replaySpeed = (float)atof(&a[4]);
		}
		else if (startsWith(a, "-rp="))
		{
			g_commandLineOptions.replayDataLog = true;
			g_commandLineOptions.logPath = &a[4];
		}
		else if (!strncmp(a, "-r", 2))
		{
			g_commandLineOptions.replayDataLog = true;
		}
		else
		{
			cout << "Unrecognized command line option: " << a << endl;
			cltool_outputUsage();
			return false;
		}
	}

	// We are either using a serial port or replaying data
	if ((g_commandLineOptions.comPort.length() == 0) && !g_commandLineOptions.replayDataLog)
	{
		cltool_outputUsage();
		return false;
	}

	if (g_commandLineOptions.bootloaderFileName.length() && g_commandLineOptions.comPort.length() == 0)
		cout << "Use COM_PORT option \"-c=\" with bootloader" << endl;

	return true;
}

bool cltool_replayDataLog()
{
	if (g_commandLineOptions.logPath.length() == 0)
	{
		cout << "Please specify the replay log path!" << endl;
		return false;
	}

	cISLogger logger;
	if (!logger.LoadFromDirectory(g_commandLineOptions.logPath))
	{
		cout << "Failed to load log files: " << g_commandLineOptions.logPath << endl;
		return false;
	}

	cout << "Replaying log files: " << g_commandLineOptions.logPath << endl;
	p_data_t *data;
	while ((data = logger.ReadData()) != NULL)
	{
		g_inertialSenseDisplay.ProcessData(data, g_commandLineOptions.replayDataLog, g_commandLineOptions.replaySpeed);
	}

	cout << "Done replaying log files: " << g_commandLineOptions.logPath << endl;
	g_inertialSenseDisplay.Goodbye();
	return true;
}

int cltool_runBootloader(const char* port, const char* fileName, const char* verifyFileName)
{
	cout << "Updating firmware to: " << g_commandLineOptions.bootloaderFileName << endl;

	// for debug
	// SERIAL_PORT_DEFAULT_TIMEOUT = 9999999;

	char errorBuffer[1024];
	if (!serialPortPlatformInit(&g_serialPort))
	{
		printf("Failed to initialize serial port\r\n");
		return -3;
	}

	serialPortSetPort(&g_serialPort, port);
	if (!enableBootloader(&g_serialPort, errorBuffer, sizeof(errorBuffer)))
	{
		printf("Failed to enable bootloader: %s\r\n", errorBuffer);
		serialPortClose(&g_serialPort);
		return -2;
	}

	printf("Erasing flash...\r\n");
	bootload_params_t params;
	params.fileName = fileName;
	params.port = &g_serialPort;
	params.error = errorBuffer;
	params.errorLength = sizeof(errorBuffer) / sizeof(errorBuffer[0]);
	params.obj = NULL;
	params.uploadProgress = bootloadUploadProgress;
	params.verifyProgress = bootloadVerifyProgress;
	params.verifyFileName = verifyFileName;
	if (!bootloadFileEx(&params))
	{
		printf("Error in bootloader: %s\r\n", errorBuffer);
		serialPortClose(&g_serialPort);
		return -1;
	}

	serialPortClose(&g_serialPort);

	return 0;
}

void cltool_outputUsage()
{
	cout << boldOff;
	cout << "-----------------------------------------------------------------" << endlbOn;
	cout << "                    $ Inertial Sense CL Tool" << endl;
	cout << boldOn << "USAGE" << endlbOn;
	cout << "    " << APP_NAME << APP_EXT << boldOff << " [OPTION]" << endlbOn;
	cout << endlbOn;
	cout << "EXAMPLE USAGE" << endlbOn;
	cout << "    " << APP_NAME << APP_EXT << " -c=" <<     EXAMPLE_PORT << " -sINS1             " << EXAMPLE_SPACE_1 << boldOff << " # stream one data set" << endlbOn;
	cout << "    " << APP_NAME << APP_EXT << " -c=" <<     EXAMPLE_PORT << " -sINS2 -sGPS -sBaro" << EXAMPLE_SPACE_1 << boldOff << " # stream multiple sets" << endlbOn;
	cout << "    " << APP_NAME << APP_EXT << " -c=" <<     EXAMPLE_PORT << " -sINS2 -l -lts=0   " << EXAMPLE_SPACE_1 << boldOff << " # stream and log data" << endlbOn;
	cout << "    " << APP_NAME << APP_EXT << " -rp=" << EXAMPLE_LOG_DIR                                              << boldOff << " # replay data file" << endlbOn;
	cout << "    " << APP_NAME << APP_EXT << " -c=" <<     EXAMPLE_PORT << " -b=" << EXAMPLE_FIRMWARE_FILE           << boldOff << " # bootload firmware" << endlbOn;
	cout << endlbOn;
	cout << "DESCRIPTION" << endlbOff;
	cout << "    Command line utility for communicating, logging, and updating" << endl;
	cout << "    firmware with Inertial Sense product line." << endl;
	cout << endlbOn;
	cout << "OPTIONS" << endlbOn;
	cout << "    -c=" << boldOff << "COM_PORT    select the serial port" << endlbOn;
	cout << endlbOn;
	cout << "    -b=" << boldOff << "FILEPATH    bootload firmware using .hex file FILEPATH" << endlbOn;
	cout << endlbOn;
	cout << "    -h" << boldOff << "             display this help menu" << endlbOn;
	cout << "    -scroll" << boldOff << "        scroll displayed messages to show history" << endlbOn;
	cout << endlbOn;
	cout << "    -sINS1" << boldOff << "         stream message DID_INS_1" << endlbOn;
	cout << "    -sINS2" << boldOff << "         stream message DID_INS_2" << endlbOn;
	cout << "    -sDualIMU" << boldOff << "      stream message DID_DUAL_IMU" << endlbOn;
	cout << "    -sIMU1" << boldOff << "         stream message DID_IMU_1" << endlbOn;
	cout << "    -sIMU2" << boldOff << "         stream message DID_IMU_2" << endlbOn;
	cout << "    -sGPS " << boldOff << "         stream message DID_GPS" << endlbOn;
	cout << "    -sMag1" << boldOff << "         stream message DID_MAGNETOMETER_1" << endlbOn;
	cout << "    -sBaro" << boldOff << "         stream message DID_BAROMETER" << endlbOn;
	cout << "    -sSol " << boldOff << "         stream solution messages (IMU, GPS, INS2, etc.)" << endlbOn;
	cout << "    -sSensors" << boldOff << "      stream message DID_SYS_SENSORS" << endlbOn;
	cout << "    -sDThetaVel" << boldOff << "    stream message DID_DELTA_THETA_VEL" << endlbOn;

	cout << endl << "    Logging is enabled by default." << endlbOn;
	cout << "    -loff" << boldOff << "          disable logging" << endlbOn;
	cout << "    -lp=" << boldOff << "PATH       log data to path" << endlbOn;
	cout << "    -r" << boldOff << "             replay data log from default path" << endlbOn;
	cout << "    -rp=" << boldOff << "PATH       replay data log from PATH" << endlbOn;
	cout << "    -rs=" << boldOff << "SPEED      replay data log at x SPEED" << endlbOn;
	cout << "    -lms=" << boldOff << "MBYTES    log max space in MB (default: 1024)" << endlbOn;
	cout << "    -lmf=" << boldOff << "BYTES     log max file size in bytes (default: 5242880)" << endlbOn;
	cout << "    -lmm=" << boldOff << "BYTES     log max memory in bytes (default: 131072)" << endlbOn;
	cout << "    -lts=" << boldOff << "0         log use timestamp sub folder" << endlbOn;

	cout << boldOff;   // Last line.  Leave bold text off on exit.
}

void cltool_setupCtrlCHandler()
{

#if defined(_WIN32)

	SetConsoleCtrlHandler((PHANDLER_ROUTINE)ctrlHandler, true);

#else

	// register for Ctrl-C signal
	signal(SIGINT, signalFunction);

#endif

}