/*
 * buffer_mode.h
 *
 *  Created on: Nov 3, 2018
 *      Author: biobot
 */

#ifndef LIB_BIOBOT_BIOBOT_TRAJECTORY_BUFFER_MODE_BUFFER_MODE_H_
#define LIB_BIOBOT_BIOBOT_TRAJECTORY_BUFFER_MODE_BUFFER_MODE_H_


#include <stdint.h>



/*
 * 	Buffer Mode Compact
 *
 * 	Only store the rampup in the buffer
 */
enum class BmCompactType : uint8_t {
	SINGLE,
	CONSTANT,
	RAMPUP_RAMPDOWN,
	FULL
};

struct bm_compact {
	BmCompactType mode;
	uint32_t rampup_len;
	uint32_t plateau_freq;
	uint32_t plateau_len;
	uint32_t rampdown_idx;
	uint32_t buf_size;			// Length of actual data in buffer
	bool is_converted = false;	// Flag that indicate if the frequencies have been converted to the hardware
	uint32_t steps;
};


/*
 * 	Buffer Mode Compact Loop
 *
 * 	Store the rampup and rampdown in the buffer
 */
enum class BmCompactLoopType : uint8_t {
	SINGLE,
	CONSTANT,
	RAMPUP_RAMPDOWN,
	FULL
};

struct bm_compact_loop {
	BmCompactLoopType mode;
	uint32_t rampup_len;
	uint32_t plateau_freq;
	uint32_t plateau_len;
	uint32_t rampdown_len;
};


/*
 * 	Buffer Mode Normal
 *
 * 	Store the rampup, plateau and rampdown in the buffer
 */

struct bm_normal {
	uint32_t buf_len;
};


#endif /* LIB_BIOBOT_BIOBOT_TRAJECTORY_BUFFER_MODE_BUFFER_MODE_H_ */
