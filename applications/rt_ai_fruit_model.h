#ifndef __RT_AI_FRUIT_MODEL_H
#define __RT_AI_FRUIT_MODEL_H

#include <fruit.h>
#include <fruit_data.h>

/* model info ... */

// model name
#define RT_AI_FRUIT_MODEL_NAME			"fruit"
#define RT_AI_FRUIT_WORK_BUFFER_BYTES		AI_FRUIT_DATA_ACTIVATIONS_SIZE
#define RT_AI_FRUIT_DATA_WEIGHTS_SIZE		AI_FRUIT_DATA_WEIGHTS_SIZE

#define RT_AI_FRUIT_BUFFER_ALIGNMENT		AI_FRUIT_ACTIVATIONS_ALIGNMENT

#define RT_AI_FRUIT_IN_NUM				AI_FRUIT_IN_NUM
#define RT_AI_FRUIT_IN_1_SIZE				AI_FRUIT_IN_1_SIZE
#define RT_AI_FRUIT_IN_1_SIZE_BYTES		AI_FRUIT_IN_1_SIZE_BYTES
#define RT_AI_FRUIT_IN_TOTAL_SIZE_BYTES	(AI_FRUIT_IN_1_SIZE_BYTES)
#define RT_AI_FRUIT_IN_SIZE_BYTES			{AI_FRUIT_IN_1_SIZE_BYTES}


#define RT_AI_FRUIT_OUT_NUM				AI_FRUIT_OUT_NUM
#define RT_AI_FRUIT_OUT_1_SIZE			AI_FRUIT_OUT_1_SIZE
#define RT_AI_FRUIT_OUT_1_SIZE_BYTES		AI_FRUIT_OUT_1_SIZE_BYTES
#define RT_AI_FRUIT_OUT_TOTAL_SIZE_BYTES	(AI_FRUIT_OUT_1_SIZE_BYTES)
#define RT_AI_FRUIT_OUT_SIZE_BYTES		{AI_FRUIT_OUT_1_SIZE_BYTES}


#define RT_AI_FRUIT_TOTAL_BUFFER_SIZE		(NULL)		//unused

#endif	//end