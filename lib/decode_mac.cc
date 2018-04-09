/*
 * Copyright (C) 2013, 2016 Bastian Bloessl <bloessl@ccs-labs.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <ieee802-11/decode_mac.h>

#include "utils.h"
#include "viterbi_decoder/viterbi_decoder.h"

#include <boost/crc.hpp>
#include <gnuradio/io_signature.h>

using namespace gr::ieee802_11;

#define LINKTYPE_IEEE802_11 105 /* http://www.tcpdump.org/linktypes.html */

class decode_mac_impl : public decode_mac {

public:
decode_mac_impl(bool log, bool debug) :
	block("decode_mac",
			gr::io_signature::make(1, 1, 48),
			gr::io_signature::make2(2, 2, 64 * sizeof(gr_complex), 80 * sizeof(gr_complex))),
	d_log(log),
	d_debug(debug),
	d_snr(0),
	d_noise_floor(0),
	d_nom_freq(0.0),
	d_freq_offset(0.0),
	d_ofdm(BPSK_1_2),
	d_frame(d_ofdm, 0),
	d_frame_complete(true) {

	message_port_register_out(pmt::mp("out"));
	set_tag_propagation_policy(block::TPP_DONT);
}

int general_work (int noutput_items, gr_vector_int& ninput_items,
		gr_vector_const_void_star& input_items,
		gr_vector_void_star& output_items) {

	const uint8_t *in = (const uint8_t*)input_items[0];
    gr_complex *out_f = (gr_complex*) output_items[0];
    gr_complex *out_t = (gr_complex*) output_items[1];

	int i = 0;

	std::vector<gr::tag_t> tags;
	const uint64_t nread = this->nitems_read(0);

	dout << "Decode MAC: input " << ninput_items[0] << std::endl;
    dout << "Decode MAC: d_symbols_offset = " << d_symbols_offset
         << ", d_symbols_len = " << d_symbols_len << std::endl;

	int o = std::min(noutput_items, d_symbols_len - d_symbols_offset);

    if (o > 0) {
        std::cout << "Decode MAC: publish " << o << " symbols\n";

        std::memcpy(out_t, t_inputs + 80 * d_symbols_offset, o * 80 * sizeof(gr_complex));
        std::memcpy(out_f, f_inputs + 64 * d_symbols_offset, o * 64 * sizeof(gr_complex));

        if(d_symbols_offset == 0) {
            pmt::pmt_t dict = pmt::make_dict();
            dict = pmt::dict_add(dict, pmt::mp("packet_len"), pmt::from_long(length_f_in/64));
            dict = pmt::dict_add(dict, pmt::mp("No."), pmt::from_long(d_frame_rx_num));
            add_item_tag(0, nitems_written(0),
                    pmt::string_to_symbol("packet_len"),
                    dict,
                    pmt::string_to_symbol(name()));
            dout << "Decode MAC: add tag \"packet_len="
                << length_f_in / 64 << "\" to Port [0]" << std::endl;

            dict = pmt::make_dict();
            dict = pmt::dict_add(dict, pmt::mp("wifi_start"), pmt::from_long(length_t_in));
            dict = pmt::dict_add(dict, pmt::mp("No."), pmt::from_long(d_frame_rx_num));
            add_item_tag(1, nitems_written(0),
                    pmt::string_to_symbol("wifi_start"),
                    dict,
                    pmt::string_to_symbol(name()));
            dout << "Decode MAC: add tag \"wifi_start="
                << length_t_in << "\" to Port [1]" << std::endl;
        }

        d_symbols_offset += o;

        if(d_symbols_offset == d_symbols_len) {
            d_symbols_offset = 0;
            d_symbols_len = 0;
            d_frame_rx_num ++;
            std::cout << "Decode MAC: d_frame_rx_num = " << d_frame_rx_num << std::endl;
        }
        dout << "Decode MAC: output " << o << " symbols\n";
        return o;
    }

	while(i < ninput_items[0]) {

        get_tags_in_range(tags, 0, nread + i, nread + i + 1,
            pmt::string_to_symbol("wifi_last"));

		if(tags.size()) {
			pmt::pmt_t pair = tags[0].value;

            length_t_in = pmt::blob_length(pmt::cdr(pair)) / sizeof(gr_complex);
            const gr_complex *t_in = reinterpret_cast<const gr_complex *>(pmt::blob_data(pmt::cdr(pair)));
            memcpy(t_inputs, t_in, length_t_in * sizeof(gr_complex));

            length_f_in = pmt::blob_length(pmt::car(pair)) / sizeof(gr_complex);
            const gr_complex *f_in = reinterpret_cast<const gr_complex *>(pmt::blob_data(pmt::car(pair)));
            memcpy(f_inputs, f_in, length_f_in * sizeof(gr_complex));

            if (length_t_in / 80 != length_f_in / 64) {
                std::cout << ">>> Warning: Decode MAC: length_t_in mismatches length_f_in" << std::endl;
            }
            dout << "DECODE MAC: capture \"wifi_last\" tag: received " << length_t_in << " t samples, and "
              << length_f_in << " f samples" << std::endl;
        }

		get_tags_in_range(tags, 0, nread + i, nread + i + 1,
			pmt::string_to_symbol("wifi_start"));

		if(tags.size()) {
			if (d_frame_complete == false) {
				dout << "Warning: starting to receive new frame before old frame was complete" << std::endl;
				dout << "Already copied " << copied << " out of " << d_frame.n_sym << " symbols of last frame" << std::endl;
			}
            d_symbols_len = 0;
            d_symbols_offset = 0;
			d_frame_complete = false;

			pmt::pmt_t dict = tags[0].value;
			int len_data = pmt::to_uint64(pmt::dict_ref(dict, pmt::mp("frame_bytes"), pmt::from_uint64(MAX_PSDU_SIZE+1)));
			int encoding = pmt::to_uint64(pmt::dict_ref(dict, pmt::mp("encoding"), pmt::from_uint64(0)));
			d_snr = pmt::to_double(pmt::dict_ref(dict, pmt::mp("snr"), pmt::from_double(0)));
			d_noise_floor = pmt::to_double(pmt::dict_ref(dict, pmt::mp("noise_floor"), pmt::from_double(0)));
			d_nom_freq = pmt::to_double(pmt::dict_ref(dict, pmt::mp("freq"), pmt::from_double(0)));
			d_freq_offset = pmt::to_double(pmt::dict_ref(dict, pmt::mp("freq_offset"), pmt::from_double(0)));

			ofdm_param ofdm = ofdm_param((Encoding)encoding);
			frame_param frame = frame_param(ofdm, len_data);

			// check for maximum frame size
			if(frame.n_sym <= MAX_SYM && frame.psdu_size <= MAX_PSDU_SIZE) {
				d_ofdm = ofdm;
				d_frame = frame;
				copied = 0;
				dout << "Decode MAC: frame start -- len " << len_data
					<< "  symbols " << frame.n_sym << "  encoding "
					<< encoding << std::endl;
			} else {
				dout << "Dropping frame which is too large (symbols or bits)" << std::endl;
			}
		}

		if(copied < d_frame.n_sym) {
			// dout << "copy one symbol, copied " << copied << " out of " << d_frame.n_sym << std::endl;
			std::memcpy(d_rx_symbols + (copied * 48), in, 48);
			copied++;

			if(copied == d_frame.n_sym) {
				dout << "received complete frame - decoding" << std::endl;
				decode();
				in += 48;
				i++;
                d_frame_complete = true;
                if (decode_success) {
                    d_symbols_len = length_f_in / 64;
                    std::cout << "Decode MAC: decode successful, d_symbols_len = " << d_symbols_len << std::endl;
                }
				break;
			}
		}

		in += 48;
		i++;
	}

	consume(0, i);

	return 0;
}

void forecast (int noutput_items, gr_vector_int &ninput_items_required) {

	if(d_symbols_len) {
		ninput_items_required[0] = 0;
	}
    else {
		ninput_items_required[0] = noutput_items;
	}
}

void decode() {
    decode_success = false;
	for(int i = 0; i < d_frame.n_sym * 48; i++) {
		for(int k = 0; k < d_ofdm.n_bpsc; k++) {
			d_rx_bits[i*d_ofdm.n_bpsc + k] = !!(d_rx_symbols[i] & (1 << k));
		}
	}

	deinterleave();
	uint8_t *decoded = d_decoder.decode(&d_ofdm, &d_frame, d_deinterleaved_bits);
	descramble(decoded);
	print_output();

	// skip service field
	boost::crc_32_type result;
	result.process_bytes(out_bytes + 2, d_frame.psdu_size);
	if(result.checksum() != 558161692) {
		dout << "checksum wrong -- dropping" << std::endl;
		return;
	}

	mylog(boost::format("encoding: %1% - length: %2% - symbols: %3%")
			% d_ofdm.encoding % d_frame.psdu_size % d_frame.n_sym);

	// create PDU
	pmt::pmt_t blob = pmt::make_blob(out_bytes + 2, d_frame.psdu_size - 4);
	pmt::pmt_t enc = pmt::from_uint64(d_ofdm.encoding);
	pmt::pmt_t dict = pmt::make_dict();
	dict = pmt::dict_add(dict, pmt::mp("encoding"), enc);
	dict = pmt::dict_add(dict, pmt::mp("snr"), pmt::from_double(d_snr));
	dict = pmt::dict_add(dict, pmt::mp("noise_floor"), pmt::from_double(d_noise_floor));
	dict = pmt::dict_add(dict, pmt::mp("nomfreq"), pmt::from_double(d_nom_freq));
	dict = pmt::dict_add(dict, pmt::mp("freqofs"), pmt::from_double(d_freq_offset));
	dict = pmt::dict_add(dict, pmt::mp("dlt"), pmt::from_long(LINKTYPE_IEEE802_11));
	dict = pmt::dict_add(dict, pmt::mp("crc_included"), pmt::PMT_F);    // for connection with mapper
	message_port_pub(pmt::mp("out"), pmt::cons(dict, blob));
    d_frame_num ++;
    std::cout << "Decode MAC: d_frame_num = " << d_frame_num << std::endl;
    decode_success = true;
}

void deinterleave() {

	int n_cbps = d_ofdm.n_cbps;
	int first[n_cbps];
	int second[n_cbps];
	int s = std::max(d_ofdm.n_bpsc / 2, 1);

	for(int j = 0; j < n_cbps; j++) {
		first[j] = s * (j / s) + ((j + int(floor(16.0 * j / n_cbps))) % s);
	}

	for(int i = 0; i < n_cbps; i++) {
		second[i] = 16 * i - (n_cbps - 1) * int(floor(16.0 * i / n_cbps));
	}

	int count = 0;
	for(int i = 0; i < d_frame.n_sym; i++) {
		for(int k = 0; k < n_cbps; k++) {
			d_deinterleaved_bits[i * n_cbps + second[first[k]]] = d_rx_bits[i * n_cbps + k];
		}
	}
}


void descramble (uint8_t *decoded_bits) {

	int state = 0;
	std::memset(out_bytes, 0, d_frame.psdu_size+2);

	for(int i = 0; i < 7; i++) {
		if(decoded_bits[i]) {
			state |= 1 << (6 - i);
		}
	}
	out_bytes[0] = state;

	int feedback;
	int bit;

	for(int i = 7; i < d_frame.psdu_size*8+16; i++) {
		feedback = ((!!(state & 64))) ^ (!!(state & 8));
		bit = feedback ^ (decoded_bits[i] & 0x1);
		out_bytes[i/8] |= bit << (i%8);
		state = ((state << 1) & 0x7e) | feedback;
	}
}

void print_output() {

	dout << std::endl;
	dout << "psdu size" << d_frame.psdu_size << std::endl;
	for(int i = 2; i < d_frame.psdu_size+2; i++) {
		dout << std::setfill('0') << std::setw(2) << std::hex << ((unsigned int)out_bytes[i] & 0xFF) << std::dec << " ";
		if(i % 16 == 15) {
			dout << std::endl;
		}
	}
	dout << std::endl;
	for(int i = 2; i < d_frame.psdu_size+2; i++) {
		if((out_bytes[i] > 31) && (out_bytes[i] < 127)) {
			dout << ((char) out_bytes[i]);
		} else {
			dout << ".";
		}
	}
	dout << std::endl;
}

private:
	bool d_debug;
	bool d_log;

	frame_param d_frame;
	ofdm_param d_ofdm;
	double d_snr;  // dB
	double d_noise_floor;  // dB
	double d_nom_freq;  // nominal frequency, Hz
	double d_freq_offset;  // frequency offset, Hz
	viterbi_decoder d_decoder;

	uint8_t d_rx_symbols[48 * MAX_SYM];
	uint8_t d_rx_bits[MAX_ENCODED_BITS];
	uint8_t d_deinterleaved_bits[MAX_ENCODED_BITS];
	uint8_t out_bytes[MAX_PSDU_SIZE + 2]; // 2 for signal field

	int copied;
	bool d_frame_complete;

    // TODO: replace 100 with MAX_SYM
    gr_complex f_inputs[64*100];
    gr_complex t_inputs[80*100];
    int length_t_in;
    int length_f_in;
	int d_symbols_offset = 0;
	int d_symbols_len = 0;
    bool decode_success;
    int d_frame_num = 0;
    int d_frame_rx_num = 0;
};

decode_mac::sptr
decode_mac::make(bool log, bool debug) {
	return gnuradio::get_initial_sptr(new decode_mac_impl(log, debug));
}
