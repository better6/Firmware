float64 lat			# target position (deg * 1e7)
float64 lon			# target position (deg * 1e7) 
float32 alt			# target position
float32 vy			# target vel in y
float32 vx			# target vel in x
float32 vz			# target vel in z 
uint8 est_cap       # target reporting capabilities
uint8 slave_delay   # Receive delay parameters passed by the host

uint64 master_utc   #记录下接收到的主机的utc时间
uint8 form_type      # 主机传递给从机的编队类型
