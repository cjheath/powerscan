#include	<stdio.h>
#include	<stdint.h>
#include	<inttypes.h>

#include	<SoapySDR/Device.h>
#include	<SoapySDR/Formats.h>

#include	<fftw3.h>

typedef	int_least64_t	Frequency;
typedef	int_least64_t	ClockTime;

#define	FFT_MAX_BITS	16			// 65536 element FFT at most
#define	MIN_DWELL_TIME	100000			// minimum time on each tuning, in microseconds
#define	MAX_CROP_RATIO	0.6			// Cropping more than this just doesn't make sense
#define	RETUNE_USLEEP	5000			// 5ms. Why is this not built-in to Soapy?
#define	MAX_SAMPLES 	(01<<FFT_MAX_BITS)	// Maximum number of I/Q sample pairs to receive in each buffer

typedef struct
{
	const char*	sdr_name;		// The name of an SDR device known to SoapySDR
	int		sdr_channel;
//	const char*	antenna;		// Which antenna to use
	int		gain;			// db of gain to use

	Frequency	start_frequency;	// Lowest frequency to report
	Frequency	end_frequency;		// Highest frequency to report
	Frequency	frequency_resolution;	// Size of frequency step to report
	Frequency	requested_sample_rate;	// Don't try to go faster than this, if set

	int		repetition_limit;	// Number of times to scan the range (0 = continuous)
	int		scan_time;		// Number of seconds for one scan (default = 10)

	float		crop_ratio;		// How much of each tuning range should we discard?

	FILE*		verbose;		// Where to send verbose output (NULL means don't)

	int		web_port;		// port to listen on localhost, 0 for no web UI

	/* Calculated or discovered configuration settings */
	SoapySDRDevice*	device;
	size_t		channel_count;		// How many channels are available on this device?
	double*		sample_rates;		// Available sample rates
	size_t		num_sample_rates;
	double		sample_rate;		// Selected sample rate

	/* Runtime variables */
	SoapySDRStream*	stream;

	int		tuning_count;		// Number of times we have to retune for one scan
	int		dwell_time;		// Number of microseconds for each tuning
	Frequency	tuning_start;		// Initial centre frequency to tune
	Frequency	tuning_bandwidth;	// Bandwidth to digitise
	Frequency	current_frequency;	// Current frequency tuned

	/* FFT variables */
	int		fft_size;		
	fftwf_complex*	fftw_in;		// FFT input buffer
	fftwf_complex*	fftw_out;		// FFT output buffer. [0] is DC, then center to min-freq = max-freq back to centre 
	fftwf_plan	fftw_plan;		// FFTW's plan
	float*		window;			// FFT Window function
	int		fft_fill;		// Next fftw_in slot to fill
	float*		fft_power;		// Power per frequency for this FFT

	float*		power_accumulation;	// Accumulated power over the entire scan (all tunings)
	long		accumulation_count;	// How many times have we accumulated power (across all tunings)
	int		power_buckets;		// Number of accumulated power buckets over the entire scan

	int_least64_t	last_time;		// Returned buffer timestamp or clock time received
	int_least64_t	first_time;		// First returned buffer timestamp or clock time received
} ProgramConfiguration;

void		setup_webserver(ProgramConfiguration* pc);
void		stop_webserver();
