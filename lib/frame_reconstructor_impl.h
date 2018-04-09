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

#ifndef INCLUDED_IEEE802_11_FRAME_RECONSTRUCTOR_IMPL_H
#define INCLUDED_IEEE802_11_FRAME_RECONSTRUCTOR_IMPL_H

#include <ieee802-11/frame_equalizer.h>     // use Enum Equalizer
#include <ieee802-11/frame_reconstructor.h>
#include <ieee802-11/constellations.h>
#include "equalizer/base.h"
#include "viterbi_decoder/viterbi_decoder.h"

namespace gr {
  namespace ieee802_11 {

    class frame_reconstructor_impl : virtual public frame_reconstructor
    {

      public:
        frame_reconstructor_impl(Equalizer algo, double freq, double bw, bool log, bool debug);
        ~frame_reconstructor_impl();

        void set_algorithm(Equalizer algo);
        void set_bandwidth(double bw);
        void set_frequency(double freq);

        void forecast (int noutput_items, gr_vector_int &ninput_items_required);
        int general_work(int noutput_items,
            gr_vector_int &ninput_items,
            gr_vector_const_void_star &input_items,
            gr_vector_void_star &output_items);

      private:


        bool parse_signal(uint8_t *signal);
        bool decode_signal_field(uint8_t *rx_bits);
        void deinterleave(uint8_t *rx_bits);

        equalizer::base *d_equalizer;
        gr::thread::mutex d_mutex;
        std::vector<gr::tag_t> tags;
        bool d_debug;
        bool d_log;
        int  d_current_symbol;
        viterbi_decoder d_decoder;

        // freq offset
        double d_freq;  // Hz
        double d_freq_offset_from_synclong;  // Hz, estimation from "sync_long" block
        double d_bw;  // Hz
        double d_er;
        double d_epsilon0;
        gr_complex d_prev_pilots[4];

        int  d_frame_bytes;
        int  d_frame_symbols;
        int  d_frame_encoding;

        uint8_t d_deinterleaved[48];
        gr_complex symbols[48];

        boost::shared_ptr<gr::digital::constellation> d_frame_mod;
        constellation_bpsk::sptr d_bpsk;
        constellation_qpsk::sptr d_qpsk;
        constellation_16qam::sptr d_16qam;
        constellation_64qam::sptr d_64qam;

        static const int interleaver_pattern[48];

        void rx_in(pmt::pmt_t msg);
        gr_complex d_H[64];     // CSI value
        gr_complex ltf0[64];    // store the first LTF
        gr_complex *symbol_inputs;   // pointers to the symbol inputs (LTF not included)
        int d_frame_length_from_msg;
        void print_csi(void);
        long packet_len;
        int d_frame_num = 0;
    };

  } // namespace ieee802_11
} // namespace gr

#endif /* INCLUDED_IEEE802_11_FRAME_RECONSTRUCTOR_IMPL_H */
