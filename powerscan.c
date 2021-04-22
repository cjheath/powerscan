/*
 * powerscan: Measure a power spectrum from a SoapySDR receiver
 */
#include	<getopt.h>
#include	<stdlib.h>
#include	<stdio.h>
#include	<string.h>
#include	<stdbool.h>
#include	<stdint.h>
#include	<inttypes.h>
#include	<complex.h>
#include	<math.h>
#include	<assert.h>
#include	<signal.h>

#include	<SoapySDR/Device.h>
#include	<SoapySDR/Formats.h>

#include	<fftw3.h>

#ifdef _WIN32
#include	<windows.h>
#include	<fcntl.h>
#include	<io.h>
#include	"getopt/getopt.h"
#define		usleep(x)	Sleep(x/1000)
#define	_USE_MATH_DEFINES
#else
#include	<unistd.h>
#include	<sys/time.h>			// For gettimeofday()
#endif

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

ProgramConfiguration	config;
int		signals_caught;

// Function prototypes:
bool		scan(ProgramConfiguration* pc);
bool		retune(ProgramConfiguration* pc, Frequency frequency);
bool		flush_data_after_config_change(ProgramConfiguration* pc);
bool		receive_block(ProgramConfiguration* pc, Frequency frequency);
void		process_buffer(ProgramConfiguration* pc, int16_t* buf16, int samples);
void		handle_fft_out(ProgramConfiguration* pc);
void		print_soapy_flags(FILE* fp, int flags);
void		list_sdr_devices(FILE* fp);
void		list_device_capabilities(ProgramConfiguration* pc);
void		list_sample_rates(ProgramConfiguration* pc);
void		select_sample_rate(ProgramConfiguration* pc);
const char*	s_if_plural(int i) { return i != 1 ? "s" : ""; }
bool		initialise_configuration(ProgramConfiguration* pc);
void		list_channel_variables(ProgramConfiguration* pc);
void		plan_tuning(ProgramConfiguration* pc);
const char*	setup_stream(ProgramConfiguration* pc);
void		finalise_configuration(ProgramConfiguration* pc);
bool		plan_fft(ProgramConfiguration* pc);
void		setup_interrupts();
void		interrupt_request(void);
ClockTime	clock_time();
void		default_parameters(ProgramConfiguration* pc);
Frequency	frequency_from_str(const char* cp);
void		usage(int exit_code);
bool		gather_parameters(ProgramConfiguration* pc, int argc, char **argv);

int main(int argc, char **argv)
{
	ProgramConfiguration* pc = &config;

	if (!gather_parameters(pc, argc, argv)	// Sort out the provided parameters
	 || !initialise_configuration(pc))	// Figure out how to use them
		usage(0);

	for (int repetition = 0; pc->repetition_limit == 0 || repetition < pc->repetition_limit; repetition++)
		if (!scan(pc) || signals_caught >= 1)
			break;

	finalise_configuration(pc);
	exit(0);
}

bool scan(ProgramConfiguration* pc)
{
	SoapySDRDevice_setSampleRate(pc->device, SOAPY_SDR_RX, pc->sdr_channel, pc->sample_rate);

	ClockTime	scan_start_time = clock_time();
	Frequency	frequency = pc->tuning_start;

	for (int i = 0; i < pc->tuning_count; i++, frequency += pc->tuning_bandwidth)
	{
		if (signals_caught > 1)
			return false;
		if (!retune(pc, frequency))
			break;

		// We should have a last_time from the buffer flush during retune. If not, use the scan start time.
		ClockTime	receive_end_time = pc->last_time + pc->dwell_time;

		while (pc->last_time < receive_end_time)
			if (!receive_block(pc, frequency))
				break;
	}

	return true;
}

bool retune(ProgramConfiguration* pc, Frequency frequency)
{
	SoapySDRKwargs args = {0};
	if (0 != SoapySDRDevice_setFrequency(pc->device, SOAPY_SDR_RX, pc->sdr_channel, (double)frequency, &args))
	{
		fprintf(stderr, "Failed to set frequency %" PRId64 "Hz: %s\n", frequency, SoapySDRDevice_lastError());
		return false;
	}
	if (pc->verbose)
		fprintf(pc->verbose, "Tuned to %" PRId64 "\n", frequency);
	if (!flush_data_after_config_change(pc))
	{
		fprintf(stderr, "Error: bad retune at %" PRId64 "Hz\n", frequency);
		return false;
	}
	pc->current_frequency = frequency;
	return true;
}

// This piece of crap should be hidden deep inside SoapySDR's APIs. I refuse to dignify it with configuration parameters.
bool flush_data_after_config_change(ProgramConfiguration* pc)
{
	int16_t		waste[MAX_SAMPLES * 2] = {0};
	int		r, i;

	/* wait for settling and flush buffer */
	usleep(RETUNE_USLEEP);

	void*		buffs[] = { waste };
	int		flags = 0;
	long long	buffer_time = 0;	// in nanoseconds (if set; SoapyHackRF doesn't set it)
	long		timeout = 1000000;	// In microseconds

	for (i = 0; i < 3; i++)		// REVISIT: Why 3?
	{
		r = SoapySDRDevice_readStream(pc->device, pc->stream, buffs, MAX_SAMPLES, &flags, &buffer_time, timeout);
		if (r >= 0)
		{
			pc->last_time = flags&SOAPY_SDR_HAS_TIME ? buffer_time/1000 : clock_time();
			if (!pc->first_time)
				pc->first_time = pc->last_time;
			break;
		}
	}
	return r >= 0;
}

bool receive_block(ProgramConfiguration* pc, Frequency frequency)
{
	int16_t		buf16[MAX_SAMPLES*2];
	void*		buffers[] = {buf16};
	int		flags = 0;		// Flags received in the buffer header
	long long	buffer_time = 0;	// The timestamp on the received buffer
	long long	this_time = 0;		// In microseconds
	long		timeout = 1000000;	// Timeout on this read (microseconds)
	int		samples;

	samples = SoapySDRDevice_readStream(pc->device, pc->stream, buffers, MAX_SAMPLES, &flags, &buffer_time, timeout);
	if (samples < 0) {
		fprintf(stderr, "Error: reading stream %d\n", samples);
		return false;
	}
	this_time = flags&SOAPY_SDR_HAS_TIME ? buffer_time/1000 : clock_time();
	if (!pc->first_time)
		pc->first_time = this_time;
	if (pc->verbose)
	{
		if (0)
		{
			fprintf(pc->verbose, "Received %d samples %s time %" PRId64 " ", samples, buffer_time ? "buffer" : "clock", (int_least64_t)this_time-pc->first_time);
			print_soapy_flags(pc->verbose, flags);
			fprintf(pc->verbose, "\n");
		}

		if (0)
		{
			// Look at the dynamic range of the data
			int min = 32767, max = -32767;
			for (int s = 0; s < 2*samples; s++) {
				if (min > buf16[s])
					min = buf16[s];
				if (max < buf16[s])
					max = buf16[s];
			}
			fprintf(stderr, "min=%d, max=%d\n", min/256, max/256);
		}

		// Automatic gain control here?
	}
	pc->last_time = this_time;

	process_buffer(pc, buf16, samples);
	return true;
}

void process_buffer(ProgramConfiguration* pc, int16_t* buf16, int samples)
{
	while (samples > 0)
	{
		fftwf_complex	c = (float)buf16[0] + I*(float)buf16[1];
		// Normalise samples to 0..1, multiplied by the window function
		pc->fftw_in[pc->fft_fill] = c * pc->window[pc->fft_fill] / 32768;
		buf16 += 2;
		samples--;
		if (++pc->fft_fill >= pc->fft_size)
		{
			fftwf_execute(pc->fftw_plan);
			handle_fft_out(pc);
			pc->fft_fill = 0;
		}
	}
}

void	handle_fft_out(ProgramConfiguration* pc)
{
	for (int s = 1; s < pc->fft_size; s++)
	{
		// fftw_out[0] is DC, then center to min-freq = max-freq back to centre 
		// REVISIT: Re-order the FFT output bins from min-freq to max-freq
		pc->fft_power[s-1] = cabs(pc->fftw_out[s] /* /pc->fft_size */);
	}

	// REVISIT: Accumulate bin power variance?

	// Summarise into 80 bins for terminal output:
	if (0)
	{
		int	width = 79;
		int	stride = (pc->fft_size-1)/width;
		for (int s = 0; s < pc->fft_size-stride; s += stride)
		{
			float sum = 0;
			for (int j = 0; j < stride; j++)
				sum += pc->fft_power[s+j];
			fprintf(stderr, "%d: %d\n", s/stride, (int)floor(sum/stride));
		}
		exit(0);
	}

	Frequency	lowest_frequency_retained = (pc->current_frequency-pc->tuning_bandwidth/2);
	int		lowest_bin = (lowest_frequency_retained - pc->start_frequency)/pc->frequency_resolution;
	int		bin_count = pc->tuning_bandwidth/pc->frequency_resolution;

	if (lowest_bin < 0 || lowest_bin+bin_count > pc->power_buckets)
		return;	// Sometimes happens on interrupt

	for (int s = 0; s < bin_count; s++)
		pc->power_accumulation[lowest_bin+s] += pc->fft_power[s];
	pc->accumulation_count++;
}

void print_soapy_flags(FILE* fp, int flags)
{
	if (!fp)
		return;
	fprintf(fp, "flags=");
	for (int i = 01; i; i <<= 1)
	{
		if ((flags & i) == 0)
			continue;
		switch (i)
		{
		case SOAPY_SDR_END_BURST:
			fprintf(fp, "end-burst "); break;	// Last packet of a burst
		case SOAPY_SDR_HAS_TIME:
			fprintf(fp, "has-time "); break;
		case SOAPY_SDR_END_ABRUPT:
			fprintf(fp, "end-abrupt "); break;
		case SOAPY_SDR_ONE_PACKET:
			fprintf(fp, "one-packet "); break;
		case SOAPY_SDR_MORE_FRAGMENTS:
			fprintf(fp, "more-fragments "); break;	// not last packet of this fragment
		case SOAPY_SDR_WAIT_TRIGGER:
			fprintf(fp, "wait-trigger "); break;
		default:
			fprintf(fp, "0x%0X ", i); break;
		}
	}
}

void list_sdr_devices(FILE* fp)
{
	size_t  	kw_length;
	SoapySDRKwargs* kw_args;

	fprintf(fp, "Available devices are:\n");
	kw_args = SoapySDRDevice_enumerate(NULL, &kw_length);
	for (int i = 0; i < kw_length; i++)
	{
		fprintf(fp, "\t%d: ", i);
		for (int j = 0; j < kw_args[i].size; j++)
			fprintf(fp, "%s=%s ", kw_args[i].keys[j], kw_args[i].vals[j]);
		fprintf(fp, "\n");
	}
	SoapySDRKwargsList_clear(kw_args, kw_length);
}

// Report the capabilities of this device (save what we need):
void list_device_capabilities(ProgramConfiguration* pc)
{
	SoapySDRKwargs arg;

	arg = SoapySDRDevice_getHardwareInfo(pc->device);
	if (pc->verbose)
	{
		fprintf(pc->verbose, "SoapySDR Device capabilities:\n");
		for (int i = 0; i < arg.size; i++)
			fprintf(pc->verbose, "\t%s\t%s\n", arg.keys[i], arg.vals[i]);
		fprintf(pc->verbose, "\n");
	}
}

// Report the sample rates of this device (save what we need):
// REVISIT: Provide a command-line argument to list these:
void list_sample_rates(ProgramConfiguration* pc)
{
	pc->sample_rates = SoapySDRDevice_listSampleRates(pc->device, SOAPY_SDR_RX, pc->sdr_channel, &pc->num_sample_rates);

	if (pc->verbose)
	{
		fprintf(pc->verbose, "SoapySDR Device (Channel 0 Receive) has %zu sample rates:", pc->num_sample_rates);
		for (int i = 0; i < pc->num_sample_rates; i++)
			fprintf(pc->verbose, " %.0f", pc->sample_rates[i]);
		fprintf(pc->verbose, "\n");
	}
}

// Select the maximum sample rate
void select_sample_rate(ProgramConfiguration* pc)
{
	// Choose the highest sample rate not greater than requested
	for (int i = 0; i < pc->num_sample_rates; i++)
		if (pc->sample_rates[i] > pc->sample_rate	// Faster than we've seen
		 && (pc->requested_sample_rate == 0 || pc->sample_rates[i] <= pc->requested_sample_rate))
			pc->sample_rate = pc->sample_rates[i];
}

/*
 * Using the provided program configuration settings, figure out how we'll conduct the scan.
 * Allocates resources that are cleaned up by finalise_configuration().
 */
bool initialise_configuration(ProgramConfiguration* pc)
{
	const char*	error_p;

	// Open the SDR device requested, or list available devices if that failed:
	pc->device = SoapySDRDevice_makeStrArgs(pc->sdr_name);
	if (!pc->device)
	{
		fprintf(stderr, "SoapySDR device %s not found.\n", pc->sdr_name);
		fprintf(stderr, "SoapySDR error: %s\n", SoapySDRDevice_lastError());
		list_sdr_devices(stderr);

		return false;
	}

	// Provide verbose output if requested:
	list_device_capabilities(pc);

	list_sample_rates(pc);

	select_sample_rate(pc);

	// Limit the crop ratio to something sensible:
	if (pc->crop_ratio > MAX_CROP_RATIO)
		pc->crop_ratio = MAX_CROP_RATIO;
	else if (pc->crop_ratio < 0)
		pc->crop_ratio = 0;

	if (pc->start_frequency <= 0)
	{
		fprintf(stderr, "No start frequency was given\n");
		return false;
	}

	if (pc->end_frequency > 0 && pc->end_frequency <= pc->start_frequency)
	{
		fprintf(stderr, "Ignoring end frequency below start frequency\n");
		pc->end_frequency = 0;
	}

	if (pc->end_frequency <= 0)
	{		// Center around start frequency, maximum bandwidth
		Frequency	default_bandwidth = pc->sample_rate * (1 - pc->crop_ratio);
		pc->end_frequency = pc->start_frequency + default_bandwidth/2;
		pc->start_frequency = pc->end_frequency - default_bandwidth;
	}

	if (pc->frequency_resolution != 0
	 && floor(pc->sample_rate / pc->frequency_resolution) > MAX_SAMPLES)
	{
		fprintf(stderr, "Requested frequency resolution is too small, setting it to %ld\n", (long)floor(pc->sample_rate/MAX_SAMPLES));
		pc->frequency_resolution = 0;
	}
	if (pc->frequency_resolution == 0)
	{
		pc->frequency_resolution = floor(pc->sample_rate/MAX_SAMPLES);
		if (pc->frequency_resolution == 0)
			pc->frequency_resolution = 1;	// Do any SDRs have a sample rate below 65536 SPS?
	}

	// Display the native stream data format. We use CS16 anyway, for now.
	double		full_scale = 0;
	const char*	native_format = SoapySDRDevice_getNativeStreamFormat(pc->device, SOAPY_SDR_RX, pc->sdr_channel, &full_scale);
	fprintf(stderr, "Native stream format is %s with fullscale of %g\n", native_format, full_scale);

	// HackR LNA max is 40, VGA 62, AMP 14, total 116
	if (SoapySDRDevice_setGain(pc->device, SOAPY_SDR_RX, pc->sdr_channel, pc->gain) != 0) {
		fprintf(stderr, "Failed to set gain\n");
	}

	list_channel_variables(pc);

	plan_tuning(pc);

	error_p = setup_stream(pc);
	if (error_p)
	{
		fprintf(stderr, "Can't setup stream: %s\n", error_p);
		return false;
	}

	if (!plan_fft(pc))
		return false;

	setup_interrupts();

	return true;
}

// REVISIT: Provide command-line arguments for setting these, and a help option to list them:
void list_channel_variables(ProgramConfiguration* pc)
{
	// Fetch and list any channe; information variables for this channel:
	SoapySDRKwargs	channel_info = SoapySDRDevice_getChannelInfo(pc->device, SOAPY_SDR_RX, pc->sdr_channel);
	if (pc->verbose && channel_info.size > 0)
	{
		fprintf(pc->verbose, "%ld info items for channel %d: ", channel_info.size, pc->sdr_channel);
		for (int j = 0; j < channel_info.size; j++)
			fprintf(pc->verbose, "%s=%s ", channel_info.keys[j], channel_info.vals[j]);
		fprintf(pc->verbose, "\n");
	}
}

// Figure out how many times we need to retune to cover the requested bandwidth
void plan_tuning(ProgramConfiguration* pc)
{
	// We overscan at each end by the half the crop amount:
	Frequency	total_scan =
		pc->end_frequency
		- pc->start_frequency
		+ (Frequency)floor(pc->crop_ratio * pc->sample_rate);

	// Calculate how much bandwidth we get in each tuning, after cropping:
	pc->tuning_bandwidth = (Frequency)ceil(pc->sample_rate*(1.0 - pc->crop_ratio));
	pc->tuning_start = pc->start_frequency + pc->tuning_bandwidth/2;

	// How many times must we tune to cover the frequency range:
	pc->tuning_count = (int)ceil((double)total_scan / pc->tuning_bandwidth);

	// How long can we dwell on each tuning:
	pc->dwell_time = 1000000*pc->scan_time/pc->tuning_count;
	if (pc->dwell_time < MIN_DWELL_TIME)
		pc->dwell_time = MIN_DWELL_TIME;

	// Report the planned scan parameters:
	if (pc->repetition_limit)
		fprintf(stderr, "Scan %d time%s", pc->repetition_limit, s_if_plural(pc->repetition_limit));
	else
		fprintf(stderr, "Scan continuously");
	fprintf(stderr,
		" from %" PRId64 " to %" PRId64 " (covering %" PRId64 "Hz in steps of %" PRId64 "Hz) in %d tuning%s at %" PRId64 "bps using %" PRId64 "Hz lasting %dms\n",
		pc->start_frequency,
		pc->end_frequency,
		pc->end_frequency-pc->start_frequency,
		pc->frequency_resolution,
		pc->tuning_count,
		s_if_plural(pc->tuning_count),
		(long long)pc->sample_rate,
		pc->tuning_bandwidth,
		pc->dwell_time/1000
	);
}

bool plan_fft(ProgramConfiguration* pc)
{
	pc->fft_size = pc->sample_rate / pc->frequency_resolution;
	pc->fft_size = 8192;
	pc->fft_fill = 0;
	if (pc->fft_size < 4)
		pc->fft_size = 4;
	pc->frequency_resolution = pc->sample_rate/pc->fft_size;

	pc->power_buckets = (pc->end_frequency - pc->start_frequency + pc->frequency_resolution-1)/pc->frequency_resolution;

	fprintf(stderr, "Sample Rate\t%g\n", pc->sample_rate);
	fprintf(stderr, "FFT Size\t%d\n", pc->fft_size);
	fprintf(stderr, "Frequency Resolution \t%" PRId64 "\n", pc->frequency_resolution);
	fprintf(stderr, "Power buckets\t%d\n", pc->power_buckets);

	pc->fftw_in = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex) * pc->fft_size);
	pc->fftw_out = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex) * pc->fft_size);
	pc->window = (float*)fftwf_malloc(sizeof(float) * pc->fft_size);
	pc->fft_power = (float*)malloc(sizeof(float) * pc->fft_size);
	pc->power_accumulation = (float*)calloc(pc->power_buckets, sizeof(float));
	pc->accumulation_count = 0;
	if (!pc->fftw_in
	 || !pc->fftw_out
	 || !pc->window
	 || !pc->fft_power
	 || !pc->power_accumulation)
	{
		fprintf(stderr, "Unable to allocate FFT memory\n");
		return false;
	}

	pc->fftw_plan = fftwf_plan_dft_1d(pc->fft_size, pc->fftw_in, pc->fftw_out, FFTW_FORWARD, FFTW_MEASURE);
fprintf(stderr, "FFT Size %d, in=%p, out=%p\n", pc->fft_size, pc->fftw_in, pc->fftw_out);

	// Populate the window function
	for (int s = 0; s < pc->fft_size; s++) {
		pc->window[s] = (float) (0.5f * (1.0f - cos(2 * M_PI * s / (pc->fft_size - 1))));
	}

	return true;
}

// Set up a receiver data stream on the specified channel
const char* setup_stream(ProgramConfiguration* pc)
{
	SoapySDRKwargs	stream_args = {0};
	size_t		sdr_channel = pc->sdr_channel;

	pc->channel_count = SoapySDRDevice_getNumChannels(pc->device, SOAPY_SDR_RX);
	if ((size_t)pc->sdr_channel >= pc->channel_count)
	{
		fprintf(stderr, "Device has only %zu channel%s\n", pc->channel_count, s_if_plural(pc->channel_count));
		return "Invalid channel selected";
	}

#if SOAPY_SDR_API_VERSION < 0x00080000
	// REVISIT: This Soapy API prints an [INFO] message without being asked
	if (SoapySDRDevice_setupStream(pc->device, &pc->stream, SOAPY_SDR_RX, SOAPY_SDR_CS16, &sdr_channel, 1, &stream_args) != 0)
		return SoapySDRDevice_lastError();
#else
	pc->stream = SoapySDRDevice_setupStream(pc->device, SOAPY_SDR_RX, SOAPY_SDR_CS16, &sdr_channel, 1, &stream_args);
	if (pc->stream == NULL)
		return SoapySDRDevice_lastError();
#endif

	SoapySDRDevice_activateStream(pc->device, pc->stream, 0, 0, 0);	// flags, timeout, numElems (burst control)
	// REVISIT: direct dsampling
	// REVISIT: offset tuning
	return 0;
}

void finalise_configuration(ProgramConfiguration* pc)
{
	if (pc->stream)
	{
		SoapySDRDevice_deactivateStream(pc->device, pc->stream, 0, 0);
		SoapySDRDevice_closeStream(pc->device, pc->stream);
		pc->stream = 0;
	}
	SoapySDRDevice_unmake(pc->device);
	pc->device = 0;
}

#ifdef _WIN32
BOOL WINAPI
interrupt_handler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		interrupt_request();
		return TRUE;
	}
	return FALSE;
}
#else
void interrupt_handler(int signum)
{
	interrupt_request();
}
#endif

void interrupt_request(void)
{
	fprintf(stderr, "Signal caught, %s.\n", signals_caught++ == 0 ? "finishing" : "abort");
}

void setup_interrupts()
{
#ifdef _WIN32
	SetConsoleCtrlHandler((PHANDLER_ROUTINE)interrupt_handler, TRUE);
#else
	struct sigaction sa;
	sa.sa_handler = interrupt_handler;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = 0;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	sigaction(SIGQUIT, &sa, NULL);
	sigaction(SIGPIPE, &sa, NULL);
	signal(SIGPIPE, SIG_IGN);
#endif
}

ClockTime clock_time()
{
	struct timeval	tv;
	gettimeofday(&tv, (void*)0);

	return tv.tv_sec*1000000 + tv.tv_usec;
}

void default_parameters(ProgramConfiguration* pc)
{
	memset(pc, 0, sizeof(*pc));
	pc->crop_ratio = 0.25;
	pc->scan_time = 10;
}

Frequency frequency_from_str(const char* cp)
{
	char*		endptr = 0;
	double		d = strtod(cp, &endptr);

	if (endptr == cp)
		goto invalid;

	switch (*endptr)
	{
	case 'k': case 'K': d *= 1000; break;
	case 'm': case 'M': d *= 1000000L; break;
	case 'g': case 'G': d *= 1000000000L; break;
	case '\0': break;
	default:
	invalid:
		fprintf(stderr, "Invalid frequency specification: %s\n", cp);
		return 0;	// Missing or invalid number was found
	}
	return d;
}

void usage(int exit_code)
{
	fprintf(stderr,
		"Usage: powerscan [ options... ]\n"
		"\t-v\t\tDisplay detailed information\n"
		"\t-d device\tSelect an SDR device (\"help\" for a list)\n"
		"\t-C channel\tSelect an SDR channel\n"
		"\t-s freq\t\tStart frequency\n"
		"\t-e freq\t\tEnd frequency\n"
		"\t-r freq\t\tFrequency resolution\n"
		"\t-R freq\t\tSample rate upper limit\n"
		"\t-c ratio\tCrop ratio, how much of each tuning band to ignore (0-0.6)\n"
		"\t-t time\t\tComplete each scan in this many seconds (default 10)\n"
//		"\t-a name\t\tSelect antenna\n"
		"\t-g gain\t\tReceive gainn"
		"\t-1\t\tMake a single scan\n"
		"\t-l count\tScan this many times\n"
		"\t-h\t\tThis help message\n"
	);
	exit(exit_code);
}

bool gather_parameters(ProgramConfiguration* pc, int argc, char **argv)
{
	int	opt;

	default_parameters(pc);
	while ((opt = getopt(argc, argv, "vd:C:a:g:s:e:r:c:1l:t:h?")) != -1) {
		switch (opt) {
		case 'v':		// verbose output
			pc->verbose = stderr;
			break;

		case 'd':		// SDR Device name
			pc->sdr_name = optarg;
			if (strcmp(pc->sdr_name, "help") == 0)
			{
				list_sdr_devices(stdout);
				return false;
			}
			break;

		case 'C':
			pc->sdr_channel = atol(optarg);
			break;

#if 0
		case 'a':		// Select an antenna system
			pc->antenna = optarg;
			break;
#endif

		case 'g':
			pc->gain = atol(optarg);
			break;

		case 's':
			pc->start_frequency = frequency_from_str(optarg);
			break;

		case 'e':
			pc->end_frequency = frequency_from_str(optarg);
			break;

		case 'r':
			pc->frequency_resolution = frequency_from_str(optarg);
			break;

		case 'R':
			pc->requested_sample_rate = frequency_from_str(optarg);
			break;

		case 't':
			pc->scan_time = atol(optarg);
			break;

		case 'c':
			pc->crop_ratio = atof(optarg);
			break;

		case '1':		// Make a single scan
			pc->repetition_limit = 1;
			break;

		case 'l':
			pc->repetition_limit = atol(optarg);
			break;

		case 'h':		// Give help
		case '?':
		default:
			return false;
		}
	}
	return true;
}
