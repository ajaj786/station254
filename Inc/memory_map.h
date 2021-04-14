/**
  ******************************************************************************
  * @file    memory_map.h
  * @author  Mojtaba Sepehrian
  * @version V1.0
  * @date    23-November-2016
  * @brief
  ******************************************************************************
*/

#ifndef MEMORY_MAP_H_
#define MEMORY_MAP_H_


#define SYS_CONFIG_REG_BASE					(0)
#define SYS_CONFIG_REG_COUNT				(0)

#define SYS_WORKING_REG_BASE				(SYS_CONFIG_REG_BASE + SYS_CONFIG_REG_COUNT)
#define SYS_WORKING_REG_COUNT				(256)

#define	TOTAL_REG_COUNT						(SYS_WORKING_REG_BASE + SYS_WORKING_REG_COUNT)	// Maximum = 65536

//##################################################################################################################################################################
#define SEGMENT_ID(i)						(SYS_WORKING_REG_BASE + 0 + i)
#define SEGMENT_ID_COUNT					(40)

#define SEGMENT_LENGTH(i)					(SYS_WORKING_REG_BASE + 40 + i)
#define SEGMENT_LENGTH_COUNT				(40)

#define WAYPOINT_CODE_FROM(i)				(SYS_WORKING_REG_BASE + 80 + i)
#define WAYPOINT_CODE_FROM_COUNT			(40)

#define WAYPOINT_CODE_TO(i)					(SYS_WORKING_REG_BASE + 120 + i)
#define WAYPOINT_CODE_TO_COUNT				(40)
//##################################################################################################################################################################

#endif /* MEMORY_MAP_H_ */
