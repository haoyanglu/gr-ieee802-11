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

#include "frame_truncate_impl.h"
#include "utils.h"
#include <gnuradio/io_signature.h>

namespace gr {
    namespace ieee802_11 {

        frame_truncate::sptr
            frame_truncate::make(int n, const std::string &len_tag_key, bool debug) {
                return gnuradio::get_initial_sptr
                    (new frame_truncate_impl(n, len_tag_key, debug));
            }


        frame_truncate_impl::frame_truncate_impl(int n, const std::string &len_tag_key, bool debug) :
            tagged_stream_block("frame_truncate",
                    gr::io_signature::make(1, 1, sizeof(gr_complex)),
                    gr::io_signature::make(1, 1, sizeof(gr_complex)), len_tag_key),
            d_n(n), d_current_sample(0), d_debug(debug), d_frame_samples(0) {
            }

        frame_truncate_impl::~frame_truncate_impl() {
        }

        int
            frame_truncate_impl::calculate_output_stream_length(const gr_vector_int &ninput_items) {
                return std::max(0, ninput_items[0] - d_n);
            }

        int
            frame_truncate_impl::work (int noutput_items,
                    gr_vector_int &ninput_items,
                    gr_vector_const_void_star &input_items,
                    gr_vector_void_star &output_items)
            {
                const gr_complex *in = (const gr_complex *) input_items[0];
                gr_complex *out = (gr_complex *) output_items[0];
                std::vector<tag_t> tags;

                int produced = ninput_items[0] - d_n;

                if(produced <= 0)
                    return 0;

                memcpy((void *) out, in, sizeof(gr_complex) * produced);

                dout << "Frame truncate: ninput_items[0] = " << ninput_items[0]
                    << ", produced = " << produced << std::endl;

                return produced;
            }
    } /* namespace ieee802_11 */
} /* namespace gr */
