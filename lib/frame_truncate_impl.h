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

#ifndef INCLUDED_IEEE802_11_FRAME_TRUNCATE_IMPL_H
#define INCLUDED_IEEE802_11_FRAME_TRUNCATE_IMPL_H

#include <ieee802-11/frame_truncate.h>

namespace gr {
    namespace ieee802_11 {

        class frame_truncate_impl : virtual public frame_truncate
        {
            public:
                frame_truncate_impl(int n, const std::string &len_tag_key, bool debug);
                ~frame_truncate_impl();

            protected:
            int calculate_output_stream_length(const gr_vector_int &ninput_items);

                int work(int noutput_items,
                    gr_vector_int &ninput_items,
                    gr_vector_const_void_star &input_items,
                    gr_vector_void_star &output_items);

            private:
                gr::thread::mutex d_mutex;
                std::vector<gr::tag_t> tags;
                bool d_debug;
                int d_n;
                int d_current_sample;
                int d_frame_samples;
        };

    } // namespace ieee802_11
} // namespace gr

#endif /* INCLUDED_IEEE802_11_FRAME_TRUNCATE_IMPL_H */
