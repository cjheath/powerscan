/*
 * powerscan: Measure a power spectrum from a SoapySDR receiver
 */
#include	<getopt.h>
#include	<stdlib.h>
#include	<stdio.h>
#include	<string.h>
#include	<stdbool.h>
#include	<stdint.h>
#include	<unistd.h>
#include	<inttypes.h>
#include	<math.h>

#include	<SoapySDR/Device.h>
#include	<SoapySDR/Formats.h>

typedef	int_least64_t	Frequency;

#define	FFT_MAX_BITS	16
#define	MIN_DWELL_TIME	100			// minimum time on each tuning, in milliseconds
#define	MAX_CROP_RATIO	0.6			// Cropping more than this just doesn't make sense
#define	RETUNE_USLEEP	5000			// 5ms. Why is this not built-in to Soapy?

typedef struct
{
	const char*	sdr_name;		// The name of an SDR device known to SoapySDR
	int		sdr_channel;
//	const char*	antenna;		// Which antenna to use

	Frequency	start_frequency;	// Lowest frequency to report
	Frequency	end_frequency;		// Highest frequency to report
	Frequency	frequency_resolution;	// Size of frequency step to report

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

	SoapySDRStream*	stream;

	int		tuning_count;		// Number of times we have to retune for one scan
	int		dwell_time;		// Number of seconds for each tuning
	Frequency	tuning_start;		// Initial centre frequency to tune
	Frequency	tuning_bandwidth;	// Bandwidth to digitise
} ProgramConfiguration;

ProgramConfiguration	config;

// Function prototypes:
bool		scan(ProgramConfiguration* pc);
bool		retune(ProgramConfiguration* pc, Frequency frequency);
bool		flush_data_after_config_change(ProgramConfiguration* pc);
void		list_sdr_devices(FILE* fp);
void		list_device_capabilities(ProgramConfiguration* pc);
void		list_sample_rates(ProgramConfiguration* pc);
void		select_sample_rate(ProgramConfiguration* pc);
const char*	s_if_plural(int i) { return i != 1 ? "s" : ""; }
bool		initialise_configuration(ProgramConfiguration* pc);
const char*	setup_stream(ProgramConfiguration* pc);
void		finalise_configuration(ProgramConfiguration* pc);
void		default_parameters(ProgramConfiguration* pc);
Frequency	frequency_from_str(const char* cp);
void		usage(int exit_code);
bool		gather_parameters(ProgramConfiguration* pc, int argc, char **argv);

#ifdef _WIN32
#define usleep(x) Sleep(x/1000)
#endif

int main(int argc, char **argv)
{
	ProgramConfiguration* pc = &config;

	if (!gather_parameters(pc, argc, argv)	// Sort out the provided parameters
	 || !initialise_configuration(pc))	// Figure out how to use them
		usage(0);

	for (int repetition = 0; pc->repetition_limit == 0 || repetition < pc->repetition_limit; repetition++)
		if (!scan(pc))
			break;

	finalise_configuration(pc);
	exit(0);
}

bool scan(ProgramConfiguration* pc)
{
	SoapySDRDevice_setSampleRate(pc->device, SOAPY_SDR_RX, pc->sdr_channel, pc->sample_rate);

	Frequency frequency = pc->tuning_start;
	for (int i = 0; i < pc->tuning_count; i++, frequency += pc->tuning_bandwidth)
	{
		if (!retune(pc, frequency))
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
	return true;
}

// This piece of crap should be hidden deep inside SoapySDR's APIs. I refuse to dignify it with configuration parameters.
bool flush_data_after_config_change(ProgramConfiguration* pc)
{
	static int16_t	waste[(01<<16) * sizeof(int16_t) * 2] = {0};
	int		r, i;

	/* wait for settling and flush buffer */
	usleep(RETUNE_USLEEP);

	void*		buffs[] = { waste };
	int		flags = 0;
	long long	bufferTime = 0;		// in nanoseconds
	long		timeout = 1000000;	// In microseconds

	for (i = 0; i < 3; ++i) {
		r = SoapySDRDevice_readStream(pc->device, pc->stream, buffs, (01<<16), &flags, &bufferTime, timeout);
		if (r >= 0)
			break;
	}
	return r >= 0;
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
void list_sample_rates(ProgramConfiguration* pc)
{
	pc->sample_rates = SoapySDRDevice_listSampleRates(pc->device, SOAPY_SDR_RX, pc->sdr_channel, &pc->num_sample_rates);

	if (pc->verbose)
	{
		fprintf(pc->verbose, "SoapySDR Device (Channel 0 Receive) has %zu sample rates:\n", pc->num_sample_rates);
		for (int i = 0; i < pc->num_sample_rates; i++)
			fprintf(pc->verbose, "\t%.0f\n", pc->sample_rates[i]);
	}
}

// Select the maximum sample rate
void select_sample_rate(ProgramConfiguration* pc)
{
	// REVISIT: Provide a sample rate limit configuration parameter
	for (int i = 0; i < pc->num_sample_rates; i++)
		if (pc->sample_rate < pc->sample_rates[i])
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

	list_device_capabilities(pc);

	list_sample_rates(pc);

	select_sample_rate(pc);

	if (pc->start_frequency <= 0)
	{
		fprintf(stderr, "No start frequency was given\n");
		return false;
	}

	if (pc->end_frequency <= pc->start_frequency)
	{
		// Use the SDR device bandwidth around the start frequency
		pc->end_frequency = pc->start_frequency + (Frequency)floor(pc->sample_rate/2);
		pc->start_frequency = pc->end_frequency - (Frequency)floor(pc->sample_rate);
	}

	if (pc->frequency_resolution == 0)
	{
		pc->frequency_resolution = pc->sample_rate/(01<<FFT_MAX_BITS);
		if (pc->frequency_resolution == 0)
			pc->frequency_resolution = 1;
	}

	// Limit the crop ratio to something sensible:
	if (pc->crop_ratio > MAX_CROP_RATIO)
		pc->crop_ratio = MAX_CROP_RATIO;
	else if (pc->crop_ratio < 0)
		pc->crop_ratio = 0;

	// We overscan at each end by the half the crop amount:
	Frequency	total_scan =
		pc->end_frequency
		- pc->start_frequency
		+ (Frequency)floor(pc->crop_ratio * pc->sample_rate);

	// Calculate how much bandwidth we get in each tuning, after cropping:
	pc->tuning_bandwidth = (Frequency)ceil(pc->sample_rate*(1.0 - pc->crop_ratio));
	pc->tuning_start = pc->start_frequency + pc->tuning_bandwidth/2;

	// How many times must we tune to cover the frequency range:
	pc->tuning_count = (int)ceil(total_scan / pc->tuning_bandwidth);

	// How long can we dwell on each tuning:
	pc->dwell_time = 1000*pc->scan_time/pc->tuning_count;
	if (pc->dwell_time < MIN_DWELL_TIME)
		pc->dwell_time = MIN_DWELL_TIME;

	// Report the planned scan parameters:
	if (pc->repetition_limit)
		fprintf(stderr, "Scan %d time%s", pc->repetition_limit, s_if_plural(pc->repetition_limit));
	else
		fprintf(stderr, "Scan continuously");
	fprintf(stderr,
		" from %" PRId64 " to %" PRId64 " (covering %" PRId64 "Hz in steps of %" PRId64 "Hz) in %d tuning%s each lasting %dms\n",
		pc->start_frequency,
		pc->end_frequency,
		pc->end_frequency-pc->start_frequency,
		pc->frequency_resolution,
		pc->tuning_count,
		s_if_plural(pc->tuning_count),
		pc->dwell_time
	);

	error_p = setup_stream(pc);
	if (error_p)
	{
		fprintf(stderr, "Can't setup stream: %s\n", error_p);
		return false;
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

void default_parameters(ProgramConfiguration* pc)
{
	memset(pc, 0, sizeof(*pc));
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
		"\t-s freq\t\tStart frequency\n"
		"\t-e freq\t\tEnd frequency\n"
		"\t-r freq\t\tFrequency resolution\n"
		"\t-t time\t\tComplete each scan in this many seconds\n"
//		"\t-a name\t\tSelect antenna\n"
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
	while ((opt = getopt(argc, argv, "vd:C:a:s:e:r:c:1l:h?")) != -1) {
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

		case 's':
			pc->start_frequency = frequency_from_str(optarg);
			break;

		case 'e':
			pc->end_frequency = frequency_from_str(optarg);
			break;

		case 'r':
			pc->frequency_resolution = frequency_from_str(optarg);
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
