int write_cache(const char *mem_file, int cache_sel, int core_sel, int start_addr);
int clear_data_cache(int start_addr, int end_addr);
int read_cache(const char *mem_file, int cache_sel, int core_sel, int start_addr, int end_addr);
unsigned long* read_cache_as_array(int cache_sel, int core_sel, int start_addr, int end_addr);
unsigned long long int get_error_count();
int read_ddls(const char *mem_file, int start_addr, int end_addr);
int program_heater(int heater_in, int first_in);
unsigned long long get_core_cycle(int core_sel);
void dco_ddls_setup(int dco_control, int ddls_delay);
int check_cpu_end(int rpi_clk_limit, int inject_cnt, long *inject_time, long *inject_lasting_time);
void cpu_start(unsigned long long riscv_run_cycle, int dco_control);
int run_stress_test();
int max_frequency_test();
void dco_clock_test();
int program_injector_list(int *injector_list, int injector_count);
int run_deterministic_injection_rare_test(int dco_control, int inject_length);
int make_rare_event_dir(int dco_control, float vdd_test);
int run_rare_event_test();