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

# Filename: uwicore_mac_utils.py

# This python file includes all the methods involving MAC tasks, such as carrier sensing,
# fragmentation and re-assembly tasks, generation of 802.11 compliant frames and send-to and 
# receive-from PHY layer mechanisms. The original PHY code was developed by FTW (Forschungszentrum 
# Telekommunikation Wien / Telecommunications Research Center Vienna, http://www.ftw.at). The code 
# was first presented and described in the following publication:

# P.Fuxjaeger, A. Costantini, D. Valerio, P. Castiglione, G. Zacheo, T. Zemen and F. Ricciato, "IEEE 802.11p
# Transmission Using GNURadio", in Proceedings of the IEEE Karlsruhe Workshop on Software Radios (WSR10), pp. 1-4,
# 2010.

# The Uwicore Laboratory at the University Miguel Hernandez of Elche has added additional 
# functionalities to the FTW PHY code, in particular: Carrier Sensing functions, and 
# reconfigurability of the transmission GNU radio graph to allow for the possibility to 
# transmit different MAC frames (also of varying size). In addition, the Uwicore PHY
# contribution communicates with the MAC layer, and is capable to process requests from
# the MAC to sense or to transmit a packet to the wireless medium.

# The FTW OFMD code triggers the encoding procedures and sends the complex baseband 
# signal to the USRP2 sink. The Uwicore carrier sensing function, based on the example 
# 'USRP_spectrum_sense' provided by GNU Radio, estimates the power of the signal using
# the signal's Power Spectrum Density (PSD).

# Ubiquitous Wireless Communications Research Laboratory 
# Uwicore, http://www.uwicore.umh.es
# Communications Engineering Department
# University Miguel Hernandez of Elche
# Avda de la Universidad, s/n
# 03202 Elche, Spain

# Release: April 2011

# Original FTW PHY code authors:
#	Andrea Costantini 
#	Paul Fuxjaeger (fuxjaeger@ftw.at)
#	Danilo Valerio (valerio@ftw.at)
#	Paolo Castiglione (castiglione@ftw.at)
#	Giammarco Zacheo (zacheo@ftw.at)

# Authors of the PHY added functionalities:
#	Juan R. Gutierrez-Agullo (jgutierrez@umh.es)
#	Baldomero Coll-Perales (bcoll@umh.es)
#	Dr. Javier Gozalvez (j.gozalvez@umh.es)


import math
import pickle
import random
import socket
import time


def parse_mac(pkt, pkt_info, log=False):
    """
    Parse the frame
    :param pkt: input frame (string)
    :param pkt_info: pkt info such as encoding, snr, nomfreq, freqofs, dlt (dict)
    :param log: Print input
    :return: MAC parameters
    """
    header = info = ""

    if log:
        print "The content of pkt: ", pkt

    if pkt[0] == chr(0x80):
        header = "BEACON"
        mac_duration = ord(pkt[2]) + (ord(pkt[3]) << 8)
        mac_da = pkt[4:10]  # Destination address
        mac_sa = pkt[10:16]  # Source address
        mac_bssid = pkt[16:22]  # BSS ID
        mac_seq = (ord(pkt[22]) >> 4) + (ord(pkt[23]) << 4)  # Seq-ctrl
        mac_frag = (ord(pkt[22]) & 0xf)
        timestamp = ord(pkt[24]) + (ord(pkt[25]) << 8) + (ord(pkt[26]) << 16) + (ord(pkt[27]) << 24) + \
                    (ord(pkt[28]) << 32) + (ord(pkt[29]) << 40) + (ord(pkt[30]) << 48) + (ord(pkt[31]) << 56)
        bi = ord(pkt[32]) + (ord(pkt[33]) << 8)  # Beacon interval
        capability_info = ord(pkt[34]) + (ord(pkt[35]) << 8)  # Capability info
        ssid = ""
        if len(pkt) > 36:  # optional SSID is included
            ssid_len = ord(pkt[37])
            ssid = "".join(pkt[38:(38 + ssid_len)])
        info = {"packet": pkt, "mac_duration": mac_duration, "mac_add1": mac_da, "mac_add2": mac_sa,
                "N_SEQ": mac_seq, "N_FRAG": mac_frag, "MF": mac_frag, "timestamp": timestamp, "BI": bi, "SSID": ssid}

    elif pkt[0] == chr(0xc4):
        header = "CTS"
        tx_time = ord(pkt[2]) + (ord(pkt[3]) << 8)
        mac_ra = pkt[4:10]
        info = {"packet": pkt, "tx_time": tx_time, "RX_add": mac_ra}

    elif pkt[0] == chr(0xb4):
        header = "RTS"
        tx_time = ord(pkt[2]) + (ord(pkt[3]) << 8)
        mac_ra = pkt[4:10]
        mac_ta = pkt[10:16]
        info = {"packet": pkt, "tx_time": tx_time, "RX_add": mac_ra, "TX_add": mac_ta}

    elif pkt[0] == chr(0xd4):
        header = "ACK"
        tx_time = ord(pkt[2]) + (ord(pkt[3]) << 8)
        mac_ra = pkt[4:10]
        info = {"packet": pkt, "tx_time": tx_time, "RX_add": mac_ra}

    elif pkt[0] == chr(0x08) and (pkt[1] == chr(0x00) or pkt[1] == chr(0x04) or pkt[1] == chr(0x08)):  # DATA frames
        if pkt[1] == chr(0x00):
            header = "DATA"
        elif pkt[1] == chr(0x04):
            header = "DATA_FRAG"
        elif pkt[1] == chr(0x08):
            header = "DATA_RETX"

        tx_time = ord(pkt[2]) + (ord(pkt[3]) << 8)
        mac_da = pkt[4:10]
        mac_sa = pkt[10:16]
        mac_bssid = pkt[16:22]
        mac_seq = (ord(pkt[22]) >> 4) + (ord(pkt[23]) << 4)  # Seq-ctrl
        mac_frag = (ord(pkt[22]) & 0xf)
        payload = pkt[24:]
        info = {"packet": pkt, "tx_time": tx_time, "mac_add1": mac_da, "mac_add2": mac_sa, "N_SEQ": mac_seq,
                "N_FRAG": mac_frag, "MF": mac_frag, "PAYLOAD": payload}

    # Merge pkt_info into the final dict
    if info != "":
        info.update(pkt_info)

    return create_packet(header, info)


def generate_pkt(header, t_sym, encoding, payload):
    """
    Assemble the MPDU.
    FCS (CRC32) is not included and will be appended in block mapper
    :param header: frame type
    :param t_sym: symbol time
    :param encoding:data rate
    :param payload: payload
    :return: assembled MPDU
    """
    if header == "DATA":
        info = _make_data(payload, encoding, t_sym, re_tx=False, frag=False)
    elif header == "DATA_FRAG":
        info = _make_data(payload, encoding, t_sym, re_tx=False, frag=True)
    elif header == "DATA_RETX":
        info = _make_data(payload, encoding, t_sym, re_tx=True, frag=False)
    elif header == "RTS":
        info = _make_rts(payload, encoding, t_sym)
    elif header == "CTS":
        info = _make_cts(payload, encoding, t_sym)
    elif header == "ACK":
        info = _make_ack(payload, encoding, t_sym)
    elif header == "BEACON":
        info = _make_beacon(payload, encoding, t_sym)
    else:
        raise ValueError("Invalid frame type %s" % header)

    packet = info["packet"]
    packet_len = len(packet) + 4  # 4-bytes for FCS

    MAXLEN = 2304  # TODO: need verification
    if packet_len > MAXLEN:
        raise ValueError("MPDU-length must be in [0, %d]" % MAXLEN)

    info["encoding"] = encoding

    return {"HEADER": header, "INFO": info}


def retry(count, CWmin):
    """
    Update the CWslot based on the value of the Backoff retries counter
    """
    if count == 0:
        return CWmin  # First mistake

    # Following mistakes
    CW = min(1023, CWmin * (2 ** count))
    return random.randint(CWmin, CW)


def assign_mac(node):
    """
    Assign the MAC address according to the node ID
    :param node: node id
    :return: none
    """
    return chr(0x00) + chr(0x50) + chr(0xc2) + chr(0x85) + chr(0x33) + chr(node)


def format_mac(mac_str):
    """
    Convert mac address string to HEX display
    :param mac_str: string of mac address (6-char-long)
    :return: human-readable MAC address xx:xx:xx:xx:xx:xx
    """
    return ':'.join(c.encode('hex') for c in mac_str)


def cal_sym_duration(samp_rate):
    """
    Calculate the OFDM symbol duration
    Each symbol has 80 samples (64 FFT and 16 cyclic prefix)
    :param samp_rate: USRP sample rate (Hz)
    :return: symbol duration (s)
    """
    return 80 / samp_rate


def create_packet(header, data):
    """
    Define the packet format used for crosslayer communication
    """
    packet = {"HEADER": header, "DATA": data}
    return packet


def sense_channel(port, thre=-35):
    """
    Check the channel occupancy status
    :param port: socket port
    :param thre: voltage threshold. The channel is considered BUSY if the measured voltage is larger than the threshold
    :return: 1) channel status ("OCCUPIED" or "FREE")
             2) carrier sensing processing time
             3) measured voltage
    """
    time1 = time.time()
    sensed = _send_for_response(create_packet("CCA", ""), port)
    maximo = sensed["DATA"]
    maximo_dBw = 10 * math.log10(maximo)  # Save the data in dBw
    time2 = time.time()
    return "OCCUPIED" if maximo_dBw > thre else "FREE", time2 - time1, maximo_dBw


def update_nav(timetick, nav, timeslot):
    """
    Method that keeps updated the Network Allocation Vector (NAV) of the station
    """
    if nav == 0:
        return 0

    time.sleep(timeslot)
    nav = nav - (time.time() - timetick)
    if timeslot >= nav > 0:
        time.sleep(nav)
        nav = 0
    return nav


def read_phy_response(port, header):
    """
    Check if packet with a specified type is available from PHY
    :param port: socket port connecting MAC and PHY
    :param header: packet type
    :return:
    """
    reading = _send_for_response(create_packet("TAIL", header.upper()), port)
    return "YES" if reading["HEADER"] == "YES" else "NO", reading["DATA"]


def send_wo_response(pkt, port):
    """
    Send data to socket
    :param pkt: data to be sent
    :param port: socket port number
    :return: none
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((socket.gethostname(), port))
    _send_to_port(pkt, s)
    s.close()


""" MAC <-> Upper layer (Buffer) interactions """


def remove_ul_buff_packet(port):
    """
    Removes a packet from the upper layer buffer.
    Usually is called after a packet is successfully
    sent or retransmission is disabled.
    """
    send_wo_response(create_packet("remove", ""), port)  # This message orders a packet deletion


def send_ul_buff_packet(port, packet):
    """
    Insert a packet into the upper layer buffer.
    Called after a valid DATA frame is captured.
    """
    send_wo_response(create_packet("copy", packet), port)  # This message copies the value of 'payload' in a buffer


def read_ul_buffer(port):
    """
    Check the upper layer buffer for packets to send
    """
    reading = _send_for_response(create_packet("no_packet", ""), port)
    if reading["HEADER"] == "YES":  # is a Data Packet?
        return "YES", reading["DATA"]

    if reading["HEADER"] == "BEACON":  # is a BEACON Packet?
        return "BEACON", []

    return "NO", reading["DATA"]


def _make_beacon(payload, encoding, t_sym):
    """
    Generate an 802.11 compliant Beacon frame
    It only includes mandatory parts and has a
    void frame body, i.e., SSID is not included
    https://mrncciew.com/2014/10/08/802-11-mgmt-beacon-frame/
    """
    mac_frame_ctrl = chr(0b10000000) + chr(0x00)
    mac_duration = chr(0x00) * 2
    mac_da = chr(0xff) * 6
    mac_sa = payload["address2"]
    mac_bssid = chr(0xff) * 6
    ts_str, ts_long = _cal_timestamp()
    bi = _cal_beacon_interval(payload["BI"])
    capability_info = chr(0x00) * 2
    mac_seqctrl = _cal_seq_control(payload["N_SEQ"], payload["N_FRAG"])

    # pre-assemble the MPDU (duration is filled with dummy and CRC32 is missing at this point)
    packet = mac_frame_ctrl + mac_duration + mac_da + mac_sa + mac_bssid + mac_seqctrl + ts_str + bi + capability_info

    tx_time, mac_duration = _cal_tx_time(len(packet), encoding, t_sym)

    # assemble the MPDU (now duration is correct)
    packet = mac_frame_ctrl + mac_duration + mac_da + mac_sa + mac_bssid + mac_seqctrl + ts_str + bi + capability_info

    dic = {"packet": packet, "tx_time": tx_time, "mac_duration": mac_duration,
           "mac_add1": mac_da, "mac_add2": mac_sa, "N_SEQ": payload["N_SEQ"], "N_FRAG": payload["N_FRAG"],
           "MF": 0, "timestamp": ts_long, "BI": payload["BI"]}

    return dic


def _make_cts(payload, encoding, t_sym):
    """
    Generate an 802.11 compliant CTS frame
    """
    mac_frame_ctrl = chr(0xc4) + chr(0x00)
    mac_duration = chr(0x00) * 2
    mac_ra = payload["mac_ra"]

    # pre-assemble the MPDU (duration is filled with dummy and CRC32 is missing at this point)
    packet = mac_frame_ctrl + mac_duration + mac_ra

    tx_time, _ = _cal_tx_time(len(packet), encoding, t_sym)

    # airtime of frame in microseconds
    # the additional 2 at the end of this formula is not in the 
    # standard encoding rules but in the annex G reference frame it is there!
    tx_time = tx_time + payload["duration"]
    tx_time = int(tx_time) + 1
    mac_duration = chr((tx_time >> 8) & 0xff) + chr(tx_time & 0xff)

    # assemble the MPDU (now duration is correct)
    packet = mac_frame_ctrl + mac_duration + mac_ra

    dic = {"packet": packet, "tx_time": tx_time, "mac_duration": mac_duration,
           "RX_add": mac_ra, "timestamp": payload["timestamp"]}

    return dic


def _make_rts(payload, encoding, t_sym):
    """
    Generate an 802.11 compliant RTS frame
    """
    mac_frame_ctrl = chr(0xb4) + chr(0x00)
    mac_duration = chr(0x00) * 2
    mac_ra = payload["mac_ra"]
    mac_ta = payload["mac_ta"]

    # pre-assemble the MPDU (duration is filled with dummy and CRC32 is missing at this point)
    packet = mac_frame_ctrl + mac_duration + mac_ra + mac_ta

    tx_time, _ = _cal_tx_time(len(packet), encoding, t_sym)

    # airtime of frame in microseconds
    # the additional 2 at the end of this formula is not in the
    # standard encoding rules but in the annex G reference frame it is there!
    tx_time = tx_time + payload["duration"]
    tx_time = int(tx_time) + 1
    mac_duration = chr((tx_time >> 8) & 0xff) + chr(tx_time & 0xff)

    # assemble the MPDU (now duration is correct)
    packet = mac_frame_ctrl + mac_duration + mac_ra + mac_ta

    dic = {"packet": packet, "tx_time": tx_time, "mac_duration": mac_duration,
           "RX_add": mac_ra, "TX_add": mac_ta, "timestamp": payload["timestamp"]}

    return dic


def _make_ack(payload, encoding, t_sym):
    """
    Generate an 802.11 compliant ACK frame
    """
    mac_frame_ctrl = chr(0xd4) + chr(0x00)
    mac_duration = chr(0x00) * 2
    mac_ra = payload["mac_ra"]
    mac_seqctrl = chr(0x00) * 2

    # pre-assemble the MPDU (duration is filled with dummy and CRC32 is missing at this point)
    packet = mac_frame_ctrl + mac_duration + mac_ra

    tx_time, _ = _cal_tx_time(len(packet), encoding, t_sym)

    # assemble the MPDU
    packet = mac_frame_ctrl + mac_duration + mac_ra

    dic = {"packet": packet, "tx_time": tx_time, "mac_duration": mac_duration,
           "RX_add": mac_ra, "timestamp": time.time()}

    return dic


def _make_data(payload, encoding, t_sym, re_tx=False, frag=False):
    """
    Generate an 802.11 compliant DATA frame
    :param payload:
    :param encoding:
    :param t_sym:
    :param re_tx: retransmission flag
    :param frag: fragmentation flag
    :return:
    """
    mac_frame_ctrl = chr(0x08) + chr((0x08 if re_tx else 0x00) | 0x04 if frag else 0x00)

    mac_da = payload["address1"]
    mac_sa = payload["address2"]
    mac_bssid = chr(0xff) * 6  # some BSSID mac-address
    mac_seqctrl = _cal_seq_control(payload["N_SEQ"], payload["N_FRAG"])

    packet_len = 24 + len(payload["payload"])  # CRC32 is missing at this point

    tx_time, mac_duration = _cal_tx_time(packet_len, encoding, t_sym)

    # assemble the MPDU (now duration is correct)
    packet = mac_frame_ctrl + mac_duration + mac_da + mac_sa + mac_bssid + mac_seqctrl + payload[
        "payload"]

    dic = {"packet": packet, "tx_time": tx_time, "mac_duration": mac_duration,
           "mac_add1": mac_da, "mac_add2": mac_sa, "N_SEQ": payload["N_SEQ"], "N_FRAG": payload["N_FRAG"],
           "MF": 1 if frag else 0, "PAYLOAD": payload["payload"], "timestamp": payload["timestamp"]}

    return dic


def _send_to_port(data, s):
    """
    Send data to socket
    :param data: data to be sent
    :param s: socket object
    :return: none
    """
    pkt = pickle.dumps(data, 1)
    s.send(pkt)


def _recv_from_port(s, bufsize=1024):  # TODO: consider to increase the time if the packet is large
    """
    Receive data from socket
    :param s: socket object
    :param bufsize: the maximum amount of data to be received at once
    :return: data received
    """
    pkt = s.recv(bufsize)
    info = pickle.loads(pkt)
    return info


def _send_for_response(pkt, port):
    """
    Send request to the socket and expect an answer from it
    :param pkt: data to sent
    :param port: socket port number
    :return: response from socket
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((socket.gethostname(), port))
    _send_to_port(pkt, s)
    reading = _recv_from_port(s)
    s.close()
    return reading


def next_seq_num(num):
    """
    Assigns a Sequence Number within the range [0,4095]
    """
    return (num + 1) % 4096


def _cal_seq_control(seq, frag):
    """
    Calculates the Sequence Number value in packed binary string format
    """
    return chr(((seq & 0xf) << 4) + (frag & 0xf)) + chr(seq >> 4 & 0xff)


def _cal_beacon_interval(time_tick):
    """
    Calculates the Beacon Interval value in packed binary string format
    """
    if time_tick > 67108:
        print "ERROR, Beacon interval > BI MAX (67108)"
        BI = chr(0xff) + chr(0xff)
    else:
        time_tick = int(time_tick / 1024e-6) + 1
        BI = chr(time_tick & 0xff) + chr((time_tick >> 8) & 0xff)
    return BI


def _cal_timestamp():
    """
    Calculates the timestamp value in packed binary string format
    :return: timestamp as string of length 6
    """
    x = int(time.time()) + 1
    ts = chr(x & 0xff) + chr((x >> 8) & 0xff) + chr((x >> 16) & 0xff) + chr((x >> 24) & 0xff) + \
         chr((x >> 32) & 0xff) + chr((x >> 40) & 0xff) + chr((x >> 48) & 0xff) + chr((x >> 56) & 0xff)
    return ts, x


def _cal_tx_time(payload_len, encoding, t_sym):
    """
    Calculate the duration to transmit a frame
    :param payload_len: payload length in bytes
    :param encoding: data rate (int [0, 8))
    :param t_sym: symbol duration in seconds
    :return: frame transmission time
    """
    assert encoding in range(0, 8)

    N_dbps = [24.0, 36.0, 48.0, 72.0, 96.0, 144.0, 192.0, 216.0]

    # 54 bits (16-bit for SERVICE, 32-bit for CRC and 6-bit for TAIL) need to be included
    N_sym = int(math.ceil((54 + 8 * payload_len) / N_dbps[encoding]))

    # airtime of frame in microseconds
    # the additional 2 at the end of this formula is not in the
    # standard encoding rules but in the annex G reference frame it is there!
    tx_time = int((5 + N_sym) * t_sym * 1e6) + 2
    mac_duration = chr((tx_time >> 8) & 0xff) + chr(tx_time & 0xff)

    return tx_time, mac_duration
