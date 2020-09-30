/*
 * distance.c
 *
 * Created: 9/29/2020 11:45:11 PM
 *  Author: eric
 */ 

#include "distance.h"
#include "../../../hw-libs/drivers/d_i2c.h"
#include "globals.h"

#define OPT3101_Address	0x58

static uint32_t frameDelayTimeMs = 0;
static bool timingGeneratorEnabled = false;

static void OPT3101_writeReg(uint8_t reg, uint32_t value)
{
	uint8_t data[3];
	
	data[0] = (uint8_t)(value >> 0);
	data[1] = (uint8_t)(value >> 8);
	data[2] = (uint8_t)(value >> 16);
	
	i2cWrite(OPT3101_Address, reg, 1, data, 3);      
}

static uint32_t OPT3101_readReg(uint8_t reg)
{
	uint8_t data[3];
	
	i2cRead(OPT3101_Address, reg, 1, data, 3);

	uint32_t value = data[0];
	value |= (uint32_t)data[1] << 8;
	value |= (uint32_t)data[2] << 16;
	return value;
}

static void OPT3101_setMonoshotMode(void)
{
	// MONOSHOT_FZ_CLKCNT = default
	// MONOSHOT_NUMFRAME = 1
	// MONOSHOT_MODE = 3
	OPT3101_writeReg(0x27, 0x26ac07);

	// DIS_GLB_PD_OSC = 1
	// DIS_GLB_PD_AMB_DAC = 1
	// DIS_GLB_PD_REFSYS = 1
	// (other fields default)
	OPT3101_writeReg(0x76, 0x000121);

	// POWERUP_DELAY = 95
	OPT3101_writeReg(0x26, (uint32_t)95 << 10 | 0xF);
}

static void OPT3101_setContinuousMode(void)
{
	// MONOSHOT_FZ_CLKCNT = default
	// MONOSHOT_NUMFRAME = 1
	// MONOSHOT_MODE = 0
	OPT3101_writeReg(0x27, 0x26ac04);
}

static void OPT3101_setFrameTiming(uint16_t subFrameCount)
{
	// Make sure subFrameCount is a power of 2 between 1 and 4096.
	if (subFrameCount < 1 || subFrameCount > 4096 ||
	subFrameCount & (subFrameCount - 1))
	{
		subFrameCount = 4096;
	}

	// Implement equation 6 from sbau310.pdf to calculate
	// XTALK_FILT_TIME CONST, but without floating-point operations.
	uint8_t timeConst = 0;
	while ((subFrameCount << timeConst) < 1024) { timeConst++; }

	uint32_t reg2e = OPT3101_readReg(0x2e);
	reg2e = reg2e & (~(uint32_t)0xF00000 | (uint32_t)timeConst << 20);
	OPT3101_writeReg(0x2e, reg2e);

	// Set NUM_SUB_FRAMES and NUM_AVG_SUB_FRAMES.
	OPT3101_writeReg(0x9f, (subFrameCount - 1) | (uint32_t)(subFrameCount - 1) << 12);

	// Set TG_SEQ_INT_MASK_START and TG_SEQ_INT_MASK_END according to what
	// the OPT3101 datasheet says, but it's probably not needed.
	OPT3101_writeReg(0x97, (subFrameCount - 1) | (uint32_t)(subFrameCount - 1) << 12);

	// Assuming that SUB_VD_CLK_CNT has not been changed, each sub-frame is
	// 0.25 ms.  The +3 is to make sure we round up.
	uint16_t frameTimeMs = (subFrameCount + 3) / 4;

	// Add a ~6% margin in case the OPT3101 clock is running faster.
	frameDelayTimeMs = frameTimeMs + (frameTimeMs + 15) / 16;
}

static void OPT3101_enableTimingGenerator(void)
{
	OPT3101_writeReg(0x80, 0x4e1e | 1);  // TG_EN = 1
	timingGeneratorEnabled = true;
}

static void OPT3101_disableTimingGenerator(void)
{
	OPT3101_writeReg(0x80, 0x4e1e);  // TG_EN = 0
	timingGeneratorEnabled = false;
}

static void OPT3101_startSample(void)
{
	if (!timingGeneratorEnabled)
		OPT3101_enableTimingGenerator();
		
	// Set MONOSHOT_BIT to 0 before setting it to 1, as recommended here:
	// https://e2e.ti.com/support/sensors/f/1023/p/756598/2825649#2825649
	OPT3101_writeReg(0x00, 0x000000);
	OPT3101_writeReg(0x00, 0x800000);
}

void distanceInit(void)
{
	// Wait for INIT_LOAD_DONE to be set, indicating that the OPT3101 is done loading settings from its EEPROM.
	while (!(OPT3101_readReg(3) & (1 << 8)))
	{
		vTaskDelay(1);
	}
	
	OPT3101_writeReg(0x89, 7000);      // TG_OVL_WINDOW_START = 7000
	OPT3101_writeReg(0x6e, 0x0a0000);  // EN_TEMP_CONV = 1
	OPT3101_writeReg(0x50, 0x200101);  // CLIP_MODE_FC = 1, CLIP_MODE_TEMP = 0, CLIP_MODE_OFFSET = 0

	OPT3101_setMonoshotMode();
	OPT3101_setFrameTiming(512);
}

void distanceProcess(void)
{
	OPT3101_startSample();
	vTaskDelay(frameDelayTimeMs);

	//Read distance
	uint32_t reg08 = OPT3101_readReg(0x08);

	uint8_t channelUsed = reg08 >> 18 & 3;
	if (channelUsed > 2)
		channelUsed = 2;

	uint32_t phase = reg08 & 0xFFFF;  // PHASE_OUT

	// c / (2 * 10 MHz * 0x10000) = 0.22872349395 mm ~= 14990/0x10000
	int32_t distanceMillimeters = (int32_t)phase * 14990 >> 16;
	
	//Do something with values
	g_debug.i[channelUsed] = distanceMillimeters;
}