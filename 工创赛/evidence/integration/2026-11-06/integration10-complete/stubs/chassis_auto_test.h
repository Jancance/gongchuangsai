typedef struct {unsigned active,result,cause,rotation_stop_mask;} ChassisAutoStatus;
int ChassisAuto_ReadStatus(ChassisAutoStatus *);
