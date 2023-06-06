//# 初始化PID参数
Kp = 0.5  //# 比例系数
Ki = 0.2  //# 积分系数
Kd = 0.1  //# 微分系数

target_position = 4  //# 目标位置

previous_error = 0
integral = 0

while True:
   // # 读取红外传感器的数据
    sensor_data = read_sensor_data()

   // # 计算当前位置偏差
    error = target_position - sensor_data

  //  # 计算PID控制量
    proportional = Kp * error
    integral += Ki * error
    derivative = Kd * (error - previous_error)

   // # 计算转向控制量
    control_signal = proportional + integral + derivative

    //# 控制小车转向
    if control_signal > 0:
        turn_right()
    elif control_signal < 0:
        turn_left()
    else:
        keep_forward()

    //# 更新上一次的偏差值
    previous_error = error
