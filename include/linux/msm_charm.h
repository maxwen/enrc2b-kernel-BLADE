#ifndef _ARCH_ARM_MACH_MSM_MDM_IOCTLS_H
#define _ARXH_ARM_MACH_MSM_MDM_IOCTLS_H


#define CHARM_CODE		0xCC
#define WAKE_CHARM		_IO(CHARM_CODE, 1)
#define RESET_CHARM		_IO(CHARM_CODE, 2)
#define CHECK_FOR_BOOT		_IOR(CHARM_CODE, 3, int)
#define WAIT_FOR_BOOT		_IO(CHARM_CODE, 4)
#define NORMAL_BOOT_DONE	_IOW(CHARM_CODE, 5, int)
#define RAM_DUMP_DONE		_IOW(CHARM_CODE, 6, int)
#define WAIT_FOR_RESTART	_IOR(CHARM_CODE, 7, int)

#define GET_MFG_MODE   	_IO(CHARM_CODE, 94)     /* HTC added */
#define GET_RADIO_FLAG   	_IO(CHARM_CODE, 95)     /* HTC added */
#define EFS_SYNC_TIMEOUT   _IO(CHARM_CODE, 96)     /* HTC added */
#define WAIT_FOR_PORT_RELEASE   _IO(CHARM_CODE, 97)     /* HTC added */
#define CHECK_PORT_UTIL         _IO(CHARM_CODE, 98)     /* HTC added */
#define EFS_SYNC_DONE		_IO(CHARM_CODE, 99)	/* HTC added */

enum charm_boot_type {
	CHARM_NORMAL_BOOT = 0,
	CHARM_RAM_DUMPS,
};

#endif
