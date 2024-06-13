/*
#-------------------------------------------------------------------------------
# Copyright (c) 2014 Martin Marinov.
#		2017 Henning Paul based on hackrf_transfer.c 
#			by Jared Boone and Benjamin Vernoux
# All rights reserved. This program and the accompanying materials
# are made available under the terms of the GNU Public License v3.0
# which accompanies this distribution, and is available at
# http://www.gnu.org/licenses/gpl.html
#
# Contributors:
#     Martin Marinov - initial API and implementation
#     Henning Paul - rudimentary native support for HackRF
#-------------------------------------------------------------------------------
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libhackrf/hackrf.h>

#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>

#include <iostream>
#include <complex>

#include "TSDRPlugin.h"

#include "TSDRCodes.h"

#include <stdint.h>
#include <boost/algorithm/string.hpp>

#include "errors.hpp"

#define BUFFER_SIZE 262144

namespace po = boost::program_options;

typedef struct {
	tsdrplugin_readasync_function cb;
	void *ctx;
	float *floatbuff;
	size_t buff_size;
} cb_ctx_t;

typedef enum {
	TRANSCEIVER_MODE_OFF = 0,
	TRANSCEIVER_MODE_RX = 1,
	TRANSCEIVER_MODE_TX = 2,
	TRANSCEIVER_MODE_SS = 3,
} transceiver_mode_t;

static transceiver_mode_t transceiver_mode = TRANSCEIVER_MODE_RX;
static hackrf_device* device = NULL;

unsigned int lna_gain=8, vga_gain=20;
bool amp_on = 0;

uint32_t req_freq = 105e6;
float req_gain = 1;
uint32_t req_rate = 10e6;
volatile int is_running = 0;

EXTERNC TSDRPLUGIN_API void __stdcall tsdrplugin_getName(char * name) {
	strcpy(name, "TSDR HackRF Compatible Plugin");
}

int rx_callback(hackrf_transfer* transfer) {

	if(is_running){
		cb_ctx_t *hrf_callback_ctx = (cb_ctx_t*)transfer->rx_ctx;
		size_t buff_size=hrf_callback_ctx->buff_size;
		float *floatbuff=hrf_callback_ctx->floatbuff;
		void *ctx=hrf_callback_ctx->ctx;
		tsdrplugin_readasync_function cb=hrf_callback_ctx->cb;

		//fprintf(stderr, "buff_size=%d, floatbuff=%x, ctx=%x, cb=%x\n",buff_size,floatbuff,ctx,cb);

		assert(transfer->valid_length<=BUFFER_SIZE);

		if (transfer->valid_length==buff_size){
			for(int i=0;i<transfer->valid_length;i++){
				floatbuff[i]=(float)(transfer->buffer)[i]/128.0;
			}
			cb(floatbuff, transfer->valid_length, ctx, 0);

		}
		else{
			cb(floatbuff, 0, ctx, transfer->valid_length);
			int result = hackrf_stop_rx(device);
			is_running=0;
		}

	}
	return 0;
}


EXTERNC TSDRPLUGIN_API int __stdcall tsdrplugin_init(const char * params) {
	int result;

	// simulate argv and argc
	std::string sparams(params);

	typedef std::vector< std::string > split_vector_type;

	split_vector_type argscounter;
	boost::split( argscounter, sparams, boost::is_any_of(" "), boost::token_compress_on );

	const int argc = argscounter.size()+1;
	char ** argv = (char **) malloc(argc*sizeof(char *));
	char zerothtarg[] = "TSDRPlugin_HackRF";
	argv[0] = (char *) zerothtarg;
	for (int i = 0; i < argc-1; i++)
	argv[i+1] = (char *) argscounter[i].c_str();

	//variables to be set by po
	std::string serial_number, file;
	uint32_t bw;

	//setup the program options
	po::options_description desc("Allowed options");
	desc.add_options()
	("sernum", po::value<std::string>(&serial_number)->default_value(""), "HackRF device address args")
	("rate", po::value<uint32_t>(&req_rate)->default_value(req_rate), "rate of incoming samples")
	("amp", po::value<bool>(&amp_on), "enable input amplifier")
	("bw", po::value<uint32_t>(&bw), "daughterboard IF filter bandwidth in Hz");

	po::variables_map vm;
	try {
		po::store(po::parse_command_line(argc, argv, desc), vm);
		po::notify(vm);
	} catch (std::exception const&  ex)
	{
		std::string msg(boost::str(boost::format("Error: %s\n\nTSDRPlugin_HackRF %s") % ex.what() % desc));
		RETURN_EXCEPTION(msg.c_str(), TSDR_PLUGIN_PARAMETERS_WRONG);
	}

	transceiver_mode = TRANSCEIVER_MODE_RX;
	result = hackrf_init();
	if( result != HACKRF_SUCCESS ) {
		free(argv);
		RETURN_EXCEPTION("hackrf_init() failed", TSDR_CANNOT_OPEN_DEVICE);
	}

	result = hackrf_open_by_serial(serial_number.c_str(), &device);
	if( result != HACKRF_SUCCESS ) {
		free(argv);
		RETURN_EXCEPTION("hackrf_open() failed", TSDR_CANNOT_OPEN_DEVICE);
	}

	result = hackrf_set_sample_rate(device, req_rate);
	if( result != HACKRF_SUCCESS ) {
		free(argv);
		RETURN_EXCEPTION("hackrf_set_sample_rate() failed", TSDR_CANNOT_OPEN_DEVICE);
	}

	//set the IF filter bandwidth
	if (vm.count("bw")){
		result = hackrf_set_baseband_filter_bandwidth(device, bw);
		if( result != HACKRF_SUCCESS ) {
			free(argv);
			RETURN_EXCEPTION("hackrf_baseband_filter_bandwidth_set() failed", TSDR_CANNOT_OPEN_DEVICE);
		}
	}

	result = hackrf_set_hw_sync_mode(device, 0);
	if( result != HACKRF_SUCCESS ) {
		free(argv);
		RETURN_EXCEPTION("hackrf_set_hw_sync_mode() failed", TSDR_CANNOT_OPEN_DEVICE);
	}

	result = hackrf_set_amp_enable(device, amp_on?1:0);
	if( result != HACKRF_SUCCESS ) {
		free(argv);
		RETURN_EXCEPTION("hackrf_set_amp_enable() failed", TSDR_CANNOT_OPEN_DEVICE);
	}

	result = hackrf_set_vga_gain(device, vga_gain);
	if( result != HACKRF_SUCCESS ) {
		free(argv);
		RETURN_EXCEPTION("hackrf_set_vga_gain() failed", TSDR_CANNOT_OPEN_DEVICE);
	}

	result = hackrf_set_lna_gain(device, lna_gain);
	if( result != HACKRF_SUCCESS ) {
		free(argv);
		RETURN_EXCEPTION("hackrf_set_lna_gain() failed", TSDR_CANNOT_OPEN_DEVICE);
	}

	result = hackrf_set_freq(device, req_freq);
	if( result != HACKRF_SUCCESS ) {
		free(argv);
		RETURN_EXCEPTION("hackrf_set_freq() failed", TSDR_CANNOT_OPEN_DEVICE);
	}

	fprintf(stderr, "init succeeded\n");

	free(argv);
	RETURN_OK();

	return 0; // to avoid getting warning from stupid Eclpse
}

EXTERNC TSDRPLUGIN_API uint32_t __stdcall tsdrplugin_setsamplerate(uint32_t rate) {
	if (is_running)
	return tsdrplugin_getsamplerate();

	req_rate = rate;

	int result = hackrf_set_sample_rate(device, req_rate);
	if( result != HACKRF_SUCCESS ) {
		RETURN_EXCEPTION("hackrf_set_sample_rate() failed", TSDR_CANNOT_OPEN_DEVICE);
	}

	return req_rate;
}

EXTERNC TSDRPLUGIN_API uint32_t __stdcall tsdrplugin_getsamplerate() {

	return req_rate;
}

EXTERNC TSDRPLUGIN_API int __stdcall tsdrplugin_setbasefreq(uint32_t freq) {
	req_freq = freq;

	int result = hackrf_set_freq(device, req_freq);
	if( result != HACKRF_SUCCESS ) {
		RETURN_EXCEPTION("hackrf_set_freq() failed", TSDR_CANNOT_OPEN_DEVICE);
	}

	RETURN_OK();

	return 0; // to avoid getting warning from stupid Eclpse
}

EXTERNC TSDRPLUGIN_API int __stdcall tsdrplugin_stop(void) {
	is_running = 0;
	RETURN_OK();

	return 0; // to avoid getting warning from stupid Eclpse
}

EXTERNC TSDRPLUGIN_API int __stdcall tsdrplugin_setgain(float gain) {

	// max gain is 40 dB, in steps of 8 dB
	float gain_factor = 5.0 * gain + 0.5;

	int req_lna_gain=int(gain_factor)*8;	

	//fprintf(stderr, "requested gain %f, set gain %d\n",gain,req_lna_gain);

	int result = hackrf_set_lna_gain(device, req_lna_gain);
	if( result != HACKRF_SUCCESS ) {
		RETURN_EXCEPTION("hackrf_set_lna_gain() failed", TSDR_CANNOT_OPEN_DEVICE);
	}

	return 0; // to avoid getting warning from stupid Eclpse
}

EXTERNC TSDRPLUGIN_API int __stdcall tsdrplugin_readasync(tsdrplugin_readasync_function cb, void *ctx) {
	cb_ctx_t hrf_callback_ctx;
	float *floatbuff;

	is_running = 1;

	float * buff = NULL;

	fprintf(stderr, "entered tsdrplugin_readasync()\n");

	int result = hackrf_set_sample_rate(device, req_rate);
	if( result != HACKRF_SUCCESS ) {
		RETURN_EXCEPTION("hackrf_set_sample_rate() failed", TSDR_CANNOT_OPEN_DEVICE);
	}

	result = hackrf_set_vga_gain(device, vga_gain);
	result |= hackrf_set_lna_gain(device, lna_gain);

	result = hackrf_set_freq(device, req_freq);
	if( result != HACKRF_SUCCESS ) {
		RETURN_EXCEPTION("hackrf_set_freq() failed", TSDR_CANNOT_OPEN_DEVICE);
	}


	size_t buff_size = BUFFER_SIZE;
	floatbuff = (float *)malloc(buff_size * sizeof(float));

	hrf_callback_ctx.buff_size = buff_size;
	hrf_callback_ctx.floatbuff = floatbuff;
	hrf_callback_ctx.ctx=ctx;
	hrf_callback_ctx.cb=cb;

	//fprintf(stderr, "&hrf_callback_ctx=%x, buff_size=%d, floatbuff=%x, ctx=%x, cb=%x\n",&hrf_callback_ctx,buff_size,floatbuff,ctx,cb);

	
	result = hackrf_start_rx(device, (hackrf_sample_block_cb_fn)rx_callback, &hrf_callback_ctx);
	if( result != HACKRF_SUCCESS ) {
		RETURN_EXCEPTION("hackrf_start_rx() failed", TSDR_CANNOT_OPEN_DEVICE);
	}

	while(is_running && (hackrf_is_streaming(device) == HACKRF_TRUE)){

	} done_loop:

	result = hackrf_stop_rx(device);
	if( result != HACKRF_SUCCESS ) {
		if (buff!=NULL) free(buff);
		RETURN_EXCEPTION("hackrf_stop_rx() failed", TSDR_CANNOT_OPEN_DEVICE);
	} else {
		fprintf(stderr, "hackrf_stop_rx() done\n");
	}


	if (floatbuff!=NULL) free(floatbuff);

	RETURN_OK();

	return 0; // to avoid getting warning from stupid Eclpse
}

EXTERNC TSDRPLUGIN_API void __stdcall tsdrplugin_cleanup(void) {

	int result = hackrf_close(device);
	if(result != HACKRF_SUCCESS) {
		fprintf(stderr, "hackrf_close() failed\n");
	} else {
		fprintf(stderr, "hackrf_close() done\n");
	}

	hackrf_exit();
	fprintf(stderr, "hackrf_exit() done\n");

	is_running = 0;
}
