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

# Filename: ul_traffic.py

# This script, developed by the Uwicore Laboratory at the University Miguel Hernandez of Elche, 
# simulates the arrival of different packets to transmit from the upper layers. The payload is 
# a plain ASCII string, but in future versions might be a packet from OSI upper layers, such as
# IP, TCP or UDP. More detailed information can be found at www.uwicore.umh.es/mhop-testbeds.html 
# or at the publication:

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
import random
import socket
import time
from optparse import OptionParser

from gnuradio.eng_option import eng_option

import uwicore_mac_utils as mac

if __name__ == '__main__':

    parser = OptionParser(option_class=eng_option, conflict_handler="resolve")

    parser.add_option("", "--MACport", type="int", default=8001,
                      help="Socket port of upper layer (ul_buffer), [default=%default]")
    parser.add_option("-n", "--pkt_num", type="int", default=50, help="Number of packets generated [default=%default]")
    parser.add_option("-t", "--interval", type="float", default=0.02,
                      help="Packet generation interval in seconds [default=%default]")

    (options, args) = parser.parse_args()

    interval = options.interval
    pkt_num = options.pkt_num

    print "-------------------------------"
    print " Upper Layer traffic generator"
    print " Packet Number  : %d" % pkt_num
    print " Packet interval: %s s" % interval
    print " (Ctrl + C) to exit"
    print "-------------------------------", '\n'

    while pkt_num > 0:
        '''
        Pseudo-random packets generated that are sent to the Upper layer buffer
        every 'interval' interval (in seconds).
        '''
        pkt_num -= 1
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((socket.gethostname(), options.MACport))

        num = random.randint(0, 3)
        if num == 0:
            pkt = mac.create_packet("PAYLOAD", "TEST_1")
        elif num == 1:
            pkt = mac.create_packet("PAYLOAD", "TEST_2")
        elif num == 2:
            pkt = mac.create_packet("PAYLOAD", "TEST_3")
        elif num == 3:
            pkt = mac.create_packet("PAYLOAD", "TEST_4")

        packet = pickle.dumps(pkt, 1)
        s.send(packet)
        s.close()
        time.sleep(interval)
