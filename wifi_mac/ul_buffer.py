#!/usr/bin/env python

# Copyright 2005, 2006 Free Software Foundation, Inc.

# This file is part of GNU Radio

# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.

# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.


# Projectname: uwicore@umh_80211_MAC

# Filename: ul_buffer.py

# This script emulates the Upper Layer (above MAC) functionality developed by the Uwicore Laboratory 
# at the University Miguel Hernandez of Elche. It consists of a pair of packet buffers, the first one 
# stores the packets received from the MAC and the second one stores the packets that are going
# to be sent to the wireless medium. More detailed information can be found at 
# www.uwicore.umh.es/mhop-testbeds.html or at the publication:

# J.R. Gutierrez-Agullo, B. Coll-Perales and J. Gozalvez, "An IEEE 802.11 MAC Software Defined Radio Implementation for  Experimental Wireless Communications and Networking Research", Proceedings of the 2010 IFIP/IEEE Wireless Days (WD'10), pp. 1-5, 20-22 October 2010, Venice (Italy).

# Ubiquitous Wireless Communications Research Laboratory 
# Uwicore, http://www.uwicore.umh.es
# Communications Engineering Department
# University Miguel Hernandez of Elche
# Avda de la Universidad, s/n
# 03202 Elche, Spain

# Release: April 2011

# List of Authors:
#	Juan R. Gutierrez-Agullo (jgutierrez@umh.es)
#	Baldomero Coll-Perales (bcoll@umh.es)
#	Dr. Javier Gozalvez (j.gozalvez@umh.es)


import pickle
import socket
import time
from optparse import OptionParser
from threading import Thread

from gnuradio.eng_option import eng_option

import uwicore_mpif as plcp
from buffer_lib import Buffer as buffer


# Class 'Client' to handle multiple packet arrival (from PHY traffic generator or from MAC)
class Client(Thread):
    def __init__(self, socket_client, data_client, cs, cs2, print_buffer=False):
        Thread.__init__(self)
        self.socket = socket_client
        self.data = data_client
        self.cs = cs
        self.cs2 = cs2
        self.print_buffer = print_buffer

    def run(self):
        pkt = self.socket.recv(1000)
        arrived_packet = pickle.loads(pkt)

        if "no_packet" == arrived_packet["HEADER"]:  # check whether the queue is empty
            if self.cs.isEmpty():
                x = plcp.create_packet("NO", "")
                plcp.send_to_mac(self.socket, x)
                return

            self.cs.elements.reverse()
            if self.cs.read(0) == "[beacon packet]":
                x = plcp.create_packet("BEACON", "")
                # print "Buffer has BEACON to send....."
            else:
                x = plcp.create_packet("YES", self.cs.read(0))
                # print "Buffer has a DATA to send......"
            self.cs.elements.reverse()
            plcp.send_to_mac(self.socket, x)

            return

        if "remove" == arrived_packet["HEADER"]:  # Remove a packet from the buffer
            # print "Upper buffer is set to remove a packet......"
            self.cs.elements.pop()

        elif "copy" == arrived_packet["HEADER"]:  # A packet arrives from MAC layer
            self.cs2.push(arrived_packet["DATA"])

        elif "PAYLOAD" == arrived_packet["HEADER"]:  # Payload received from Upper Layer Traffic generator (ul_traffic)
            self.cs.push(arrived_packet["DATA"])

        elif "BEACON" == arrived_packet["HEADER"]:  # Beacon request arrival
            # print "Buffer receives a BEACON from upper layer......"
            self.cs.push(arrived_packet["DATA"])

        # if self.print_buffer:  # Print the content of the buffer
        #     print "== Statistics %s ==" % time.time()
        #     print "========== TX Buffer ============"
        #     print self.cs.elements
        #     print "========== RX Buffer ============"
        #     print self.cs2.elements
        # else:  # Only print the size of the buffer
        #     print "TX [%d], RX [%d]" % (self.cs.length(), self.cs2.length())


def main():
    parser = OptionParser(option_class=eng_option, conflict_handler="resolve")

    parser.add_option("", "--MACport", type="int", default=8001,
                      help="Socket port of upper layer (ul_buffer), [default=%default]")

    (options, args) = parser.parse_args()

    print '\n', "--------------------------"
    print " Upper layer running ..."
    print "  (Ctrl + C) to exit"
    print "--------------------------", '\n'

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((socket.gethostname(), options.MACport))
    server.listen(1)

    cs = buffer()
    cs2 = buffer()

    while 1:
        socket_client, data_client = server.accept()
        threadd = Client(socket_client, data_client, cs, cs2)
        threadd.start()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
