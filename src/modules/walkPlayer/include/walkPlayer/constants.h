#ifndef CONSTANTS_H
#define CONSTANTS_H

#define VCTP_TIME VOCAB4('t','i','m','e')
#define VCTP_OFFSET VOCAB3('o','f','f')
#define VCTP_CMD_NOW VOCAB4('c','t','p','n')
#define VCTP_CMD_QUEUE VOCAB4('c','t','p','q')
#define VCTP_CMD_FILE VOCAB4('c','t','p','f')
#define VCTP_POSITION VOCAB3('p','o','s')
#define VCTP_WAIT VOCAB4('w','a','i','t')

#define ACTION_IDLE           0
#define ACTION_START          1
#define ACTION_RUNNING        2
#define ACTION_ANKLEIMPEDANCE 3

#define COM_TRAJ_NUM_COLS       9
#define POSTURAL_TRAJ_NUM_COLS  21
#define CONSTRAINTS_NUM_COLS    2

#define COM_ID           0
#define POSTURAL_ID      1
#define CONSTRAINTS_ID   2


#endif
