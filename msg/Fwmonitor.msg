#飞机的控制状态，任务状态的flags，表示任务完成状况，飞机的飞行状态，失联状态，
#飞机控制保护状态

uint16 planeID

bool fw_is_ok #飞机失联、失控以及炸鸡标志位

bool need_take_off #起飞标志位

bool need_task #需要进行任务标志位

bool need_landing #需要进行降落标志位

bool need_control_formation #需要进行编队控制

bool formation_time_complete #编队时间任务已经完成

bool formation_distance_complete #编队时间任务已经完成