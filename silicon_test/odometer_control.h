int odometer_setup(int start_odometer, int end_odometer, int odometer_control);
int odometer_load();
int odometer_meas_trig(int meas_wait_usec);
int odometer_bc_read(const char *mem_file, int start_odometer, int end_odometer);
