bool enable_accelerometer
bool enable_compass
bool enable_magnetometer
bool enable_i2c
bool enable_analog
bool mag_calibration       # 1 se sto calibrando
bool enable_reference      # 1 se accetto i riferimenti da PC104
bool timeout_reference     # 1 se mancano i riferimenti da PC104
bool ppm_main_valid        # 1 se il ricevitore "main" è riconosciuto
bool ppm_remote_enabled    # 1 se il ricevitore è attivo
bool ppm_need_zero_check   # 1 se il ricevitore è stato attivato ma non a zero di surge, quindi deve passare da zero
bool ppm_channel           # ricevitore in uso: 0 main, 1 secondario
bool ppm_secondary_valid   # 1 se il ricevitore "secondario" è riconosciuto
