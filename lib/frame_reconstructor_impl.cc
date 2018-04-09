/*
 * Copyright (C) 2016 Bastian Bloessl <bloessl@ccs-labs.org>
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

#include "frame_reconstructor_impl.h"
#include "equalizer/base.h"
#include "equalizer/comb.h"
#include "equalizer/lms.h"
#include "equalizer/ls.h"
#include "equalizer/sta.h"
#include "utils.h"
#include <gnuradio/io_signature.h>

namespace gr {
    namespace ieee802_11 {

        frame_reconstructor::sptr
            frame_reconstructor::make(Equalizer algo, double freq, double bw, bool log, bool debug) {
                return gnuradio::get_initial_sptr
                    (new frame_reconstructor_impl(algo, freq, bw, log, debug));
            }


        frame_reconstructor_impl::frame_reconstructor_impl(Equalizer algo, double freq, double bw, bool log, bool debug) :
            gr::block("frame_reconstructor",
                    gr::io_signature::make2(2, 2, 64 * sizeof(gr_complex), 64 * sizeof(gr_complex)),
                    gr::io_signature::make(1, 1, 64 * sizeof(gr_complex))), // Output: main channel signals to be removed
            d_current_symbol(0), d_log(log), d_debug(debug), d_equalizer(NULL),
            d_freq(freq), d_bw(bw), d_frame_bytes(0), d_frame_symbols(0),
            d_freq_offset_from_synclong(0.0) {

                d_bpsk = constellation_bpsk::make();
                d_qpsk = constellation_qpsk::make();
                d_16qam = constellation_16qam::make();
                d_64qam = constellation_64qam::make();

                d_frame_mod = d_bpsk;

                set_tag_propagation_policy(block::TPP_DONT);
                set_algorithm(algo);
            }

        frame_reconstructor_impl::~frame_reconstructor_impl() {
        }

        void
            frame_reconstructor_impl::set_algorithm(Equalizer algo) {
                gr::thread::scoped_lock lock(d_mutex);
                delete d_equalizer;

                switch(algo) {

                    case COMB:
                        dout << "Comb" << std::endl;
                        d_equalizer = new equalizer::comb();
                        break;
                    case LS:
                        dout << "LS" << std::endl;
                        d_equalizer = new equalizer::ls();
                        break;
                    case LMS:
                        dout << "LMS" << std::endl;
                        d_equalizer = new equalizer::lms();
                        break;
                    case STA:
                        dout << "STA" << std::endl;
                        d_equalizer = new equalizer::sta();
                        break;
                    default:
                        throw std::runtime_error("Algorithm not implemented");
                }
            }

        void
            frame_reconstructor_impl::set_bandwidth(double bw) {
                gr::thread::scoped_lock lock(d_mutex);
                d_bw = bw;
            }

        void
            frame_reconstructor_impl::set_frequency(double freq) {
                gr::thread::scoped_lock lock(d_mutex);
                d_freq = freq;
            }

        void
            frame_reconstructor_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required) {
                ninput_items_required[0] = noutput_items;
                ninput_items_required[1] = noutput_items;
            }

        int
            frame_reconstructor_impl::general_work (int noutput_items,
                    gr_vector_int &ninput_items,
                    gr_vector_const_void_star &input_items,
                    gr_vector_void_star &output_items) {

                gr::thread::scoped_lock lock(d_mutex);

                const gr_complex *recovered_in = (const gr_complex *) input_items[0];
                const gr_complex *f_in = (const gr_complex *) input_items[1];
                gr_complex *out = (gr_complex *) output_items[0];

                int i = 0;
                int i_1 = 0;
                int o = 0;
                const uint64_t nread = this->nitems_read(0);
                const uint64_t nread_1 = this->nitems_read(1);
                gr_complex symbols[48];
                gr_complex current_symbol[64];

                while((i < ninput_items[0]) && (i_1 < ninput_items[1]) && (o < noutput_items)) {

                    dout << "Frame reconstructor: d_current_symbol = " << d_current_symbol << std::endl;
                    get_tags_in_range(tags, 0, nread + i, nread + i + 1,
                        pmt::string_to_symbol("packet_len"));

                    if(tags.size()) {
                        if (d_current_symbol == 0 || d_current_symbol == d_frame_symbols + 3) {
                            packet_len = pmt::to_long(tags.front().value);
                            d_frame_symbols = packet_len - 3;
                            d_current_symbol = 0;
                            dout << "Frame Reconstructor: capture \"packet_len\" tag (" << packet_len
                                << ")" << std::endl;
                        }
                        else {
                            std::cout << ">>> Warning: Frame Reconstructor: a tag is captured while processing "
                                << d_current_symbol << " / " << d_frame_symbols + 3 << " symbols" << std::endl;
                        }
                    }

                    get_tags_in_range(tags, 1, nread_1 + i_1, nread_1 + i_1 + 1,
                        pmt::string_to_symbol("packet_len"));

                    if(tags.size()) {
                        pmt::pmt_t dict = tags.front().value;
                        // add tags
                        pmt::pmt_t key = pmt::string_to_symbol("hello");
			            int len_data = pmt::to_long(pmt::dict_ref(dict, pmt::mp("No."), pmt::from_long(0)));
                        pmt::pmt_t value = pmt::from_long(len_data);
                        pmt::pmt_t srcid = pmt::string_to_symbol(alias());
                        add_item_tag(0, nitems_written(0), key, value, srcid);
                    }
                    // not interesting -> skip
                    if(d_current_symbol > (d_frame_symbols + 2)) {
                        i++;
                        i_1++;
                        std::cout << "Frame reconstructor: not interested: d_current_symbol = " << d_current_symbol << std::endl;
                        continue;
                    }

                    if(d_current_symbol == 0) {
                        dout << "Frame Reconstructor: First of " << packet_len << " reconstructed symbols" << std::endl;
                        std::memcpy(ltf0, recovered_in + i * 64, 64*sizeof(gr_complex));
                    }

                    std::memcpy(current_symbol, f_in + 64 * i_1, 64*sizeof(gr_complex));

                    // compensate sampling offset
                    gr_complex sampling_offset;
                    for(int i = 0; i < 64; i++) {
                        sampling_offset = exp(gr_complex(0, 2*M_PI*d_current_symbol*80*(d_epsilon0 + d_er)*(i-32)/64));
                        current_symbol[i] *= sampling_offset;
                    }

                    gr_complex p = equalizer::base::POLARITY[(d_current_symbol - 2) % 127];
                    gr_complex sum =
                        (current_symbol[11] *  p) +
                        (current_symbol[25] *  p) +
                        (current_symbol[39] *  p) +
                        (current_symbol[53] * -p);

                    double beta;
                    if(d_current_symbol < 2) {
                        beta = arg(
                                current_symbol[11] -
                                current_symbol[25] +
                                current_symbol[39] +
                                current_symbol[53]);

                    } else {
                        beta = arg(
                                (current_symbol[11] *  p) +
                                (current_symbol[39] *  p) +
                                (current_symbol[25] *  p) +
                                (current_symbol[53] * -p));
                    }

                    double er = arg(
                            (conj(d_prev_pilots[0]) * current_symbol[11] *  p) +
                            (conj(d_prev_pilots[1]) * current_symbol[25] *  p) +
                            (conj(d_prev_pilots[2]) * current_symbol[39] *  p) +
                            (conj(d_prev_pilots[3]) * current_symbol[53] * -p));

                    er *= d_bw / (2 * M_PI * d_freq * 80);

                    d_prev_pilots[0] = current_symbol[11] *  p;
                    d_prev_pilots[1] = current_symbol[25] *  p;
                    d_prev_pilots[2] = current_symbol[39] *  p;
                    d_prev_pilots[3] = current_symbol[53] * -p;

                    // compensate residual frequency offset
                    for(int i = 0; i < 64; i++) {
                        current_symbol[i] *= exp(gr_complex(0, -beta));
                    }

                    // update estimate of residual frequency offset
                    if(d_current_symbol >= 2) {
                        double alpha = 0.1;
                        d_er = (1-alpha) * d_er + alpha * er;
                    }

                    // do equalization
                    uint8_t dump[48];
                    d_equalizer->equalize(current_symbol, d_current_symbol,
                            symbols, dump, d_frame_mod);

                    // extract csi
                    if(d_current_symbol == 1) {
                        memcpy(d_H, d_equalizer->get_csi(), 64 * sizeof(gr_complex));

                        // add tags
                        pmt::pmt_t key = pmt::string_to_symbol("packet_len");
                        pmt::pmt_t value = pmt::from_long(packet_len);
                        pmt::pmt_t srcid = pmt::string_to_symbol(alias());
                        add_item_tag(0, nitems_written(0), key, value, srcid);
                        dout << "Frame Reconstructor: add packet_len tag" << std::endl;
                        d_frame_num ++;
                        dout << "Frame reconstructor: already output " << d_frame_num << " frames\n";

                        // print_csi();
                        // compensate ltf
                        for (int k = 0; k < 2; k++) {
                            for (int m = 0; m < 64; m++) {
                                if((m == 32) || (m < 6) || (m > 58)) {
                                    out[m] = f_in[64 * k + m];
                                }
                                else {
                                    out[m] = recovered_in[64 * k + m] * d_H[m];
                                }
                                out[m] /= sampling_offset;
                            }
                            o++;
                            out += 64;
                        }
                    }

                    if (d_current_symbol >= 2) {
                        std::memcpy(out, recovered_in + i*64, 64*sizeof(gr_complex));
                        // apply channel distortion
                        for(int m = 0; m < 64; m ++) {
                            out[m] = recovered_in[64 * i + m];
                            if(! ((m == 32) || (m < 6) || ( m > 58)) ) {
                                out[m] *= d_H[m];
                            }
                            // apply residual frequency offset
                            out[m] /= sampling_offset;
                        }
                        o ++;
                        out += 64;

                    }

                    i++;
                    i_1++;
                    d_current_symbol++;
                }

                consume(0, i);
                consume(1, i_1);
                dout << "Frame Reconstructor: [0] consumes " << i << ", [1] consumes " << i_1 << ", o = " << o << std::endl;
                return o;
            }

        void frame_reconstructor_impl::print_csi(void) {
            dout << "csi: ";
            for(int m = 0; m < 64; m++) {
                if((m == 32) || (m < 6) || (m > 58)) {
                    continue;
                }
                dout << d_H[m] << ",";
            }
            dout << std::endl;
        }

        bool
            frame_reconstructor_impl::decode_signal_field(uint8_t *rx_bits) {

                static ofdm_param ofdm(BPSK_1_2);
                static frame_param frame(ofdm, 0);

                deinterleave(rx_bits);
                uint8_t *decoded_bits = d_decoder.decode(&ofdm, &frame, d_deinterleaved);

                return parse_signal(decoded_bits);
            }

        void
            frame_reconstructor_impl::deinterleave(uint8_t *rx_bits) {
                for(int i = 0; i < 48; i++) {
                    d_deinterleaved[i] = rx_bits[interleaver_pattern[i]];
                }
            }

        bool
            frame_reconstructor_impl::parse_signal(uint8_t *decoded_bits) {

                int r = 0;
                d_frame_bytes = 0;
                bool parity = false;
                for(int i = 0; i < 17; i++) {
                    parity ^= decoded_bits[i];

                    if((i < 4) && decoded_bits[i]) {
                        r = r | (1 << i);
                    }

                    if(decoded_bits[i] && (i > 4) && (i < 17)) {
                        d_frame_bytes = d_frame_bytes | (1 << (i-5));
                    }
                }

                if(parity != decoded_bits[17]) {
                    dout << "SIGNAL: wrong parity" << std::endl;
                    return false;
                }

                switch(r) {
                    case 11:
                        d_frame_encoding = 0;
                        d_frame_symbols = (int) ceil((16 + 8 * d_frame_bytes + 6) / (double) 24);
                        d_frame_mod = d_bpsk;
                        dout << "Encoding: 3 Mbit/s   ";
                        break;
                    case 15:
                        d_frame_encoding = 1;
                        d_frame_symbols = (int) ceil((16 + 8 * d_frame_bytes + 6) / (double) 36);
                        d_frame_mod = d_bpsk;
                        dout << "Encoding: 4.5 Mbit/s   ";
                        break;
                    case 10:
                        d_frame_encoding = 2;
                        d_frame_symbols = (int) ceil((16 + 8 * d_frame_bytes + 6) / (double) 48);
                        d_frame_mod = d_qpsk;
                        dout << "Encoding: 6 Mbit/s   ";
                        break;
                    case 14:
                        d_frame_encoding = 3;
                        d_frame_symbols = (int) ceil((16 + 8 * d_frame_bytes + 6) / (double) 72);
                        d_frame_mod = d_qpsk;
                        dout << "Encoding: 9 Mbit/s   ";
                        break;
                    case 9:
                        d_frame_encoding = 4;
                        d_frame_symbols = (int) ceil((16 + 8 * d_frame_bytes + 6) / (double) 96);
                        d_frame_mod = d_16qam;
                        dout << "Encoding: 12 Mbit/s   ";
                        break;
                    case 13:
                        d_frame_encoding = 5;
                        d_frame_symbols = (int) ceil((16 + 8 * d_frame_bytes + 6) / (double) 144);
                        d_frame_mod = d_16qam;
                        dout << "Encoding: 18 Mbit/s   ";
                        break;
                    case 8:
                        d_frame_encoding = 6;
                        d_frame_symbols = (int) ceil((16 + 8 * d_frame_bytes + 6) / (double) 192);
                        d_frame_mod = d_64qam;
                        dout << "Encoding: 24 Mbit/s   ";
                        break;
                    case 12:
                        d_frame_encoding = 7;
                        d_frame_symbols = (int) ceil((16 + 8 * d_frame_bytes + 6) / (double) 216);
                        d_frame_mod = d_64qam;
                        dout << "Encoding: 27 Mbit/s   ";
                        break;
                    default:
                        dout << "unknown encoding" << std::endl;
                        return false;
                }

                mylog(boost::format("encoding: %1% - length: %2% - symbols: %3%")
                        % d_frame_encoding % d_frame_bytes % d_frame_symbols);
                return true;
            }

        const int
            frame_reconstructor_impl::interleaver_pattern[48] = {
                0, 3, 6, 9,12,15,18,21,
                24,27,30,33,36,39,42,45,
                1, 4, 7,10,13,16,19,22,
                25,28,31,34,37,40,43,46,
                2, 5, 8,11,14,17,20,23,
                26,29,32,35,38,41,44,47};

    } /* namespace ieee802_11 */
} /* namespace gr */
