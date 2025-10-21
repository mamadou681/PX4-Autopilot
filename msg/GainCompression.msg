uint64 timestamp                # time since system start (microseconds)
uint64 timestamp_sample         # time of corresponding sensor data last used for this estimate

float32[3] compression_gains	# multiplicative gain to modify the output of the controller
float32[3] spectral_damper_hpf
float32[3] spectral_damper_out
float32[3] input
