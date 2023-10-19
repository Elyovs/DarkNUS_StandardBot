/*
 * xvr_msgs.h
 *
 *  Created on: 23 Feb 2023
 *      Author: wx
 */

#ifndef BSP_INC_XVR_MSGS_H_
#define BSP_INC_XVR_MSGS_H_

#define XVR_AIM_FLOC 0x0400
typedef struct //0x400
{
    float x_loc;
    float y_loc;
} xvr_aim_floc;

#define XVR_AIM_ILOC 0x0401
typedef struct //0x401
{
    int32_t x_loc;
    int32_t y_loc;
} xvr_aim_Iloc;

#define XVR_MVT_XFSPD 0x0410
typedef struct //0x410
{
    float x_spd;
    uint32_t time;
} xvr_mvt_xfspd;

#define XVR_MVT_YFSPD 0x0411
typedef struct //0x411
{
    float y_spd;
    uint32_t time;
} xvr_mvt_yfspd;

#define XVR_MVT_ZFSPD 0x0412
typedef struct //0x411
{
    float z_spd;
    uint32_t time;
} xvr_mvt_zfspd;
#endif /* BSP_INC_XVR_MSGS_H_ */
