#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Wifi Transceiver
# Generated: Mon Dec  4 10:45:34 2017
##################################################
import struct
import zmq

import pmt
from gnuradio import zeromq

if __name__ == '__main__':
    import ctypes
    import sys

    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print "Warning: failed to XInitThreads()"

import os
import socket
import sys
import threading
import foo
import ieee802_11
import sip
import time
import uwicore_mac_utils as mac
import uwicore_mpif as plcp
from optparse import OptionParser
from Queue import Queue
from gnuradio.eng_option import eng_option

sys.path.append(os.environ.get('GRC_HIER_PATH', os.path.expanduser('~/.grc_gnuradio')))

from PyQt4 import Qt
from PyQt4.QtCore import pyqtSlot
from gnuradio import blocks
from gnuradio import gr
from gnuradio import uhd
from gnuradio.qtgui import Range, RangeWidget
from wifi_phy_hier import wifi_phy_hier  # grc-generated hier_block

from gnuradio import qtgui


def print_msg(msg, node, log=True):
    """
    Print debug info
    :param msg: message to print
    :param node: node ID as prefix
    :param log: print flag
    :return: none
    """
    if log:
        print "[%d] %s" % (node, msg)


class wifi_transceiver(gr.top_block, Qt.QWidget):
    def __init__(self, options, hostname, msgq, wireshark=False):
        gr.top_block.__init__(self, "Wifi Transceiver")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Wifi Transceiver")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except:
            pass
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("GNU Radio", "wifi_transceiver")
        self.restoreGeometry(self.settings.value("geometry").toByteArray())

        ##################################################
        # Variables
        ##################################################
        self.tx_gain = tx_gain = options.tx_gain
        self.samp_rate = samp_rate = options.samp_rate
        self.rx_gain = rx_gain = options.rx_gain
        self.lo_offset = lo_offset = options.lo_offset
        self.freq = freq = options.freq
        self.encoding = encoding = options.encoding
        self.chan_est = chan_est = options.chan_est
        self.usrp_ip = options.usrp_ip
        self.node = options.node

        ##################################################
        # Blocks
        ##################################################
        self._samp_rate_options = [10e6, 20e6]
        self._samp_rate_labels = ["10 MHz", "20 MHz"]
        self._samp_rate_tool_bar = Qt.QToolBar(self)
        self._samp_rate_tool_bar.addWidget(Qt.QLabel("samp_rate" + ": "))
        self._samp_rate_combo_box = Qt.QComboBox()
        self._samp_rate_tool_bar.addWidget(self._samp_rate_combo_box)
        for label in self._samp_rate_labels: self._samp_rate_combo_box.addItem(label)
        self._samp_rate_callback = lambda i: Qt.QMetaObject.invokeMethod(self._samp_rate_combo_box, "setCurrentIndex",
                                                                         Qt.Q_ARG("int",
                                                                                  self._samp_rate_options.index(i)))
        self._samp_rate_callback(self.samp_rate)
        self._samp_rate_combo_box.currentIndexChanged.connect(
            lambda i: self.set_samp_rate(self._samp_rate_options[i]))
        self.top_layout.addWidget(self._samp_rate_tool_bar)
        self._lo_offset_options = (0, 6e6, 11e6,)
        self._lo_offset_labels = (
            str(self._lo_offset_options[0]), str(self._lo_offset_options[1]), str(self._lo_offset_options[2]),)
        self._lo_offset_tool_bar = Qt.QToolBar(self)
        self._lo_offset_tool_bar.addWidget(Qt.QLabel("lo_offset" + ": "))
        self._lo_offset_combo_box = Qt.QComboBox()
        self._lo_offset_tool_bar.addWidget(self._lo_offset_combo_box)
        for label in self._lo_offset_labels: self._lo_offset_combo_box.addItem(label)
        self._lo_offset_callback = lambda i: Qt.QMetaObject.invokeMethod(self._lo_offset_combo_box, "setCurrentIndex",
                                                                         Qt.Q_ARG("int",
                                                                                  self._lo_offset_options.index(i)))
        self._lo_offset_callback(self.lo_offset)
        self._lo_offset_combo_box.currentIndexChanged.connect(
            lambda i: self.set_lo_offset(self._lo_offset_options[i]))
        self.top_layout.addWidget(self._lo_offset_tool_bar)
        self._freq_options = [2412000000.0, 2417000000.0, 2422000000.0, 2427000000.0, 2432000000.0, 2437000000.0,
                              2442000000.0, 2447000000.0, 2452000000.0, 2457000000.0, 2462000000.0, 2467000000.0,
                              2472000000.0, 2484000000.0, 5170000000.0, 5180000000.0, 5190000000.0, 5200000000.0,
                              5210000000.0, 5220000000.0, 5230000000.0, 5240000000.0, 5250000000.0, 5260000000.0,
                              5270000000.0, 5280000000.0, 5290000000.0, 5300000000.0, 5310000000.0, 5320000000.0,
                              5500000000.0, 5510000000.0, 5520000000.0, 5530000000.0, 5540000000.0, 5550000000.0,
                              5560000000.0, 5570000000.0, 5580000000.0, 5590000000.0, 5600000000.0, 5610000000.0,
                              5620000000.0, 5630000000.0, 5640000000.0, 5660000000.0, 5670000000.0, 5680000000.0,
                              5690000000.0, 5700000000.0, 5710000000.0, 5720000000.0, 5745000000.0, 5755000000.0,
                              5765000000.0, 5775000000.0, 5785000000.0, 5795000000.0, 5805000000.0, 5825000000.0,
                              5860000000.0, 5870000000.0, 5880000000.0, 5890000000.0, 5900000000.0, 5910000000.0,
                              5920000000.0]
        self._freq_labels = ['  1 | 2412.0 | 11g', '  2 | 2417.0 | 11g', '  3 | 2422.0 | 11g', '  4 | 2427.0 | 11g',
                             '  5 | 2432.0 | 11g', '  6 | 2437.0 | 11g', '  7 | 2442.0 | 11g', '  8 | 2447.0 | 11g',
                             '  9 | 2452.0 | 11g', ' 10 | 2457.0 | 11g', ' 11 | 2462.0 | 11g', ' 12 | 2467.0 | 11g',
                             ' 13 | 2472.0 | 11g', ' 14 | 2484.0 | 11g', ' 34 | 5170.0 | 11a', ' 36 | 5180.0 | 11a',
                             ' 38 | 5190.0 | 11a', ' 40 | 5200.0 | 11a', ' 42 | 5210.0 | 11a', ' 44 | 5220.0 | 11a',
                             ' 46 | 5230.0 | 11a', ' 48 | 5240.0 | 11a', ' 50 | 5250.0 | 11a', ' 52 | 5260.0 | 11a',
                             ' 54 | 5270.0 | 11a', ' 56 | 5280.0 | 11a', ' 58 | 5290.0 | 11a', ' 60 | 5300.0 | 11a',
                             ' 62 | 5310.0 | 11a', ' 64 | 5320.0 | 11a', '100 | 5500.0 | 11a', '102 | 5510.0 | 11a',
                             '104 | 5520.0 | 11a', '106 | 5530.0 | 11a', '108 | 5540.0 | 11a', '110 | 5550.0 | 11a',
                             '112 | 5560.0 | 11a', '114 | 5570.0 | 11a', '116 | 5580.0 | 11a', '118 | 5590.0 | 11a',
                             '120 | 5600.0 | 11a', '122 | 5610.0 | 11a', '124 | 5620.0 | 11a', '126 | 5630.0 | 11a',
                             '128 | 5640.0 | 11a', '132 | 5660.0 | 11a', '134 | 5670.0 | 11a', '136 | 5680.0 | 11a',
                             '138 | 5690.0 | 11a', '140 | 5700.0 | 11a', '142 | 5710.0 | 11a', '144 | 5720.0 | 11a',
                             '149 | 5745.0 | 11a (SRD)', '151 | 5755.0 | 11a (SRD)', '153 | 5765.0 | 11a (SRD)',
                             '155 | 5775.0 | 11a (SRD)', '157 | 5785.0 | 11a (SRD)', '159 | 5795.0 | 11a (SRD)',
                             '161 | 5805.0 | 11a (SRD)', '165 | 5825.0 | 11a (SRD)', '172 | 5860.0 | 11p',
                             '174 | 5870.0 | 11p', '176 | 5880.0 | 11p', '178 | 5890.0 | 11p', '180 | 5900.0 | 11p',
                             '182 | 5910.0 | 11p', '184 | 5920.0 | 11p']
        self._freq_tool_bar = Qt.QToolBar(self)
        self._freq_tool_bar.addWidget(Qt.QLabel("freq" + ": "))
        self._freq_combo_box = Qt.QComboBox()
        self._freq_tool_bar.addWidget(self._freq_combo_box)
        for label in self._freq_labels: self._freq_combo_box.addItem(label)
        self._freq_callback = lambda i: Qt.QMetaObject.invokeMethod(self._freq_combo_box, "setCurrentIndex",
                                                                    Qt.Q_ARG("int", self._freq_options.index(i)))
        self._freq_callback(self.freq)
        self._freq_combo_box.currentIndexChanged.connect(
            lambda i: self.set_freq(self._freq_options[i]))
        self.top_layout.addWidget(self._freq_tool_bar)
        self._encoding_options = [0, 1, 2, 3, 4, 5, 6, 7]
        self._encoding_labels = ["BPSK 1/2", "BPSK 3/4", "QPSK 1/2", "QPSK 3/4", "16QAM 1/2", "16QAM 3/4", "64QAM 2/3",
                                 "64QAM 3/4"]
        self._encoding_group_box = Qt.QGroupBox("encoding")
        self._encoding_box = Qt.QHBoxLayout()

        class variable_chooser_button_group(Qt.QButtonGroup):
            def __init__(self, parent=None):
                Qt.QButtonGroup.__init__(self, parent)

            @pyqtSlot(int)
            def updateButtonChecked(self, button_id):
                self.button(button_id).setChecked(True)

        self._encoding_button_group = variable_chooser_button_group()
        self._encoding_group_box.setLayout(self._encoding_box)
        for i, label in enumerate(self._encoding_labels):
            radio_button = Qt.QRadioButton(label)
            self._encoding_box.addWidget(radio_button)
            self._encoding_button_group.addButton(radio_button, i)
        self._encoding_callback = lambda i: Qt.QMetaObject.invokeMethod(self._encoding_button_group,
                                                                        "updateButtonChecked", Qt.Q_ARG("int",
                                                                                                        self._encoding_options.index(
                                                                                                            i)))
        self._encoding_callback(self.encoding)
        self._encoding_button_group.buttonClicked[int].connect(
            lambda i: self.set_encoding(self._encoding_options[i]))
        self.top_layout.addWidget(self._encoding_group_box)
        self._chan_est_options = [ieee802_11.LS, ieee802_11.LMS, ieee802_11.STA, ieee802_11.COMB]
        self._chan_est_labels = ["LS", "LMS", "STA", "Linear Comb"]
        self._chan_est_group_box = Qt.QGroupBox("chan_est")
        self._chan_est_box = Qt.QHBoxLayout()

        class variable_chooser_button_group(Qt.QButtonGroup):
            def __init__(self, parent=None):
                Qt.QButtonGroup.__init__(self, parent)

            @pyqtSlot(int)
            def updateButtonChecked(self, button_id):
                self.button(button_id).setChecked(True)

        self._chan_est_button_group = variable_chooser_button_group()
        self._chan_est_group_box.setLayout(self._chan_est_box)
        for i, label in enumerate(self._chan_est_labels):
            radio_button = Qt.QRadioButton(label)
            self._chan_est_box.addWidget(radio_button)
            self._chan_est_button_group.addButton(radio_button, i)
        self._chan_est_callback = lambda i: Qt.QMetaObject.invokeMethod(self._chan_est_button_group,
                                                                        "updateButtonChecked", Qt.Q_ARG("int",
                                                                                                        self._chan_est_options.index(
                                                                                                            i)))
        self._chan_est_callback(self.chan_est)
        self._chan_est_button_group.buttonClicked[int].connect(
            lambda i: self.set_chan_est(self._chan_est_options[i]))
        self.top_layout.addWidget(self._chan_est_group_box)
        self.wifi_phy_hier_0 = wifi_phy_hier(
            bandwidth=samp_rate,
            chan_est=chan_est,
            encoding=encoding,
            frequency=freq,
            sensitivity=0.56,
        )

        self.uhd_usrp_source_0 = uhd.usrp_source(
            self.usrp_ip,
            uhd.stream_args(
                cpu_format="fc32",
                channels=range(1),
            ),
        )
        self.uhd_usrp_source_0.set_samp_rate(samp_rate)
        self.uhd_usrp_source_0.set_time_now(uhd.time_spec(time.time()), uhd.ALL_MBOARDS)
        self.uhd_usrp_source_0.set_center_freq(
            uhd.tune_request(freq, rf_freq=freq - lo_offset, rf_freq_policy=uhd.tune_request.POLICY_MANUAL), 0)
        self.uhd_usrp_source_0.set_gain(rx_gain, 0)

        self.uhd_usrp_sink_0 = uhd.usrp_sink(
            self.usrp_ip,
            uhd.stream_args(
                cpu_format="fc32",
                channels=range(1),
            ),
            'packet_len',
        )
        self.uhd_usrp_sink_0.set_samp_rate(samp_rate)
        self.uhd_usrp_sink_0.set_time_now(uhd.time_spec(time.time()), uhd.ALL_MBOARDS)
        self.uhd_usrp_sink_0.set_center_freq(
            uhd.tune_request(freq, rf_freq=freq - lo_offset, rf_freq_policy=uhd.tune_request.POLICY_MANUAL), 0)
        self.uhd_usrp_sink_0.set_gain(tx_gain, 0)

        uhd_tx_gain_range = self.uhd_usrp_sink_0.get_gain_range()
        uhd_rx_gain_range = self.uhd_usrp_source_0.get_gain_range()
        print_msg('USRP TX gain range %s' % uhd_tx_gain_range, self.node)
        print_msg('USRP RX gain range %s' % uhd_rx_gain_range, self.node)

        self._tx_gain_range = Range(uhd_tx_gain_range.start(), uhd_tx_gain_range.stop(), uhd_tx_gain_range.step(),
                                    tx_gain, 200)
        self._tx_gain_win = RangeWidget(self._tx_gain_range, self.set_tx_gain, "tx_gain", "counter_slider", float)
        self.top_layout.addWidget(self._tx_gain_win)
        self._rx_gain_range = Range(uhd_rx_gain_range.start(), uhd_rx_gain_range.stop(), uhd_rx_gain_range.step(),
                                    rx_gain, 200)
        self._rx_gain_win = RangeWidget(self._rx_gain_range, self.set_rx_gain, "rx_gain", "counter_slider", float)
        self.top_layout.addWidget(self._rx_gain_win)

        self.qtgui_time_sink_x_0 = qtgui.time_sink_f(
            1024,  # size
            samp_rate,  # samp_rate
            "",  # name
            1  # number of inputs
        )
        self.qtgui_time_sink_x_0.set_update_time(0.10)
        self.qtgui_time_sink_x_0.set_y_axis(-1, 1)

        self.qtgui_time_sink_x_0.set_y_label('Amplitude', "")

        self.qtgui_time_sink_x_0.enable_tags(-1, True)
        self.qtgui_time_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "")
        self.qtgui_time_sink_x_0.enable_autoscale(False)
        self.qtgui_time_sink_x_0.enable_grid(False)
        self.qtgui_time_sink_x_0.enable_axis_labels(True)
        self.qtgui_time_sink_x_0.enable_control_panel(False)

        if not True:
            self.qtgui_time_sink_x_0.disable_legend()

        labels = ['', '', '', '', '',
                  '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
                  "magenta", "yellow", "dark red", "dark green", "blue"]
        styles = [1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1]
        markers = [-1, -1, -1, -1, -1,
                   -1, -1, -1, -1, -1]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
                  1.0, 1.0, 1.0, 1.0, 1.0]

        for i in xrange(1):
            if len(labels[i]) == 0:
                self.qtgui_time_sink_x_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_time_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_time_sink_x_0.set_line_width(i, widths[i])
            self.qtgui_time_sink_x_0.set_line_color(i, colors[i])
            self.qtgui_time_sink_x_0.set_line_style(i, styles[i])
            self.qtgui_time_sink_x_0.set_line_marker(i, markers[i])
            self.qtgui_time_sink_x_0.set_line_alpha(i, alphas[i])

        self._qtgui_time_sink_x_0_win = sip.wrapinstance(self.qtgui_time_sink_x_0.pyqwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_time_sink_x_0_win)

        self.qtgui_const_sink_x_0 = qtgui.const_sink_c(
            48 * 10,  # size
            "",  # name
            1  # number of inputs
        )
        self.qtgui_const_sink_x_0.set_update_time(0.10)
        self.qtgui_const_sink_x_0.set_y_axis(-2, 2)
        self.qtgui_const_sink_x_0.set_x_axis(-2, 2)
        self.qtgui_const_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, "")
        self.qtgui_const_sink_x_0.enable_autoscale(False)
        self.qtgui_const_sink_x_0.enable_grid(False)
        self.qtgui_const_sink_x_0.enable_axis_labels(True)

        if not True:
            self.qtgui_const_sink_x_0.disable_legend()

        labels = ['', '', '', '', '',
                  '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1]
        colors = ["blue", "red", "red", "red", "red",
                  "red", "red", "red", "red", "red"]
        styles = [0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0]
        markers = [0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
                  1.0, 1.0, 1.0, 1.0, 1.0]
        for i in xrange(1):
            if len(labels[i]) == 0:
                self.qtgui_const_sink_x_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_const_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_const_sink_x_0.set_line_width(i, widths[i])
            self.qtgui_const_sink_x_0.set_line_color(i, colors[i])
            self.qtgui_const_sink_x_0.set_line_style(i, styles[i])
            self.qtgui_const_sink_x_0.set_line_marker(i, markers[i])
            self.qtgui_const_sink_x_0.set_line_alpha(i, alphas[i])

        self._qtgui_const_sink_x_0_win = sip.wrapinstance(self.qtgui_const_sink_x_0.pyqwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_const_sink_x_0_win)
        self.ieee802_11_parse_mac_0 = ieee802_11.parse_mac(False, False)

        if wireshark:  # save the captured packets in pcap for record in Wireshark
            self.foo_wireshark_connector_0 = foo.wireshark_connector(127, False)
            self.blocks_file_sink_0 = blocks.file_sink(gr.sizeof_char * 1, '/tmp/wifi.pcap', True)
            self.blocks_file_sink_0.set_unbuffered(True)
            self.msg_connect((self.wifi_phy_hier_0, 'mac_out'), (self.foo_wireshark_connector_0, 'in'))
            self.connect((self.foo_wireshark_connector_0, 0), (self.blocks_file_sink_0, 0))

        self.foo_packet_pad2_0 = foo.packet_pad2(False, False, 0.001, 10000, 10000)
        (self.foo_packet_pad2_0).set_min_output_buffer(100000)
        self.blocks_pdu_to_tagged_stream_0_0 = blocks.pdu_to_tagged_stream(blocks.complex_t, 'packet_len')
        self.blocks_multiply_const_vxx_0 = blocks.multiply_const_vcc((0.6,))
        (self.blocks_multiply_const_vxx_0).set_min_output_buffer(100000)
        self.zeromq_push_msg_sink_0 = zeromq.push_msg_sink("tcp://127.0.0.1:%d" % options.PHYRXport, 100)
        self.msg_sink = blocks.message_sink(4, msgq, True)  # for CCA

        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.wifi_phy_hier_0, 'carrier'), (self.blocks_pdu_to_tagged_stream_0_0, 'pdus'))
        self.msg_connect((self.wifi_phy_hier_0, 'mac_out'), (self.ieee802_11_parse_mac_0, 'in'))
        self.msg_connect((self.wifi_phy_hier_0, 'mac_out'), (self.zeromq_push_msg_sink_0, 'in'))
        self.connect((self.blocks_multiply_const_vxx_0, 0), (self.foo_packet_pad2_0, 0))
        self.connect((self.blocks_pdu_to_tagged_stream_0_0, 0), (self.qtgui_const_sink_x_0, 0))
        self.connect((self.foo_packet_pad2_0, 0), (self.uhd_usrp_sink_0, 0))
        self.connect((self.uhd_usrp_source_0, 0), (self.wifi_phy_hier_0, 0))
        self.connect((self.wifi_phy_hier_0, 0), (self.blocks_multiply_const_vxx_0, 0))
        self.connect((self.wifi_phy_hier_0.ieee802_11_moving_average_xx_0, 0), self.msg_sink)  # for CCA
        self.connect((self.wifi_phy_hier_0.blocks_divide_xx_0, 0), (self.qtgui_time_sink_x_0, 0))

    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "wifi_transceiver")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

    def get_tx_gain(self):
        return self.tx_gain

    def set_tx_gain(self, tx_gain):
        self.tx_gain = tx_gain
        self.uhd_usrp_sink_0.set_gain(self.tx_gain, 0)
        print_msg('Tx Gain -> %f dB' % (self.uhd_usrp_sink_0.get_gain()), self.node)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self._samp_rate_callback(self.samp_rate)
        self.wifi_phy_hier_0.set_bandwidth(self.samp_rate)
        self.uhd_usrp_source_0.set_samp_rate(self.samp_rate)
        self.uhd_usrp_sink_0.set_samp_rate(self.samp_rate)

    def get_rx_gain(self):
        return self.rx_gain

    def set_rx_gain(self, rx_gain):
        self.rx_gain = rx_gain
        self.uhd_usrp_source_0.set_gain(self.rx_gain, 0)
        print_msg('Rx Gain -> %f dB' % (self.uhd_usrp_source_0.get_gain()), self.node)

    def get_lo_offset(self):
        return self.lo_offset

    def set_lo_offset(self, lo_offset):
        self.lo_offset = lo_offset
        self._lo_offset_callback(self.lo_offset)
        self.uhd_usrp_source_0.set_center_freq(uhd.tune_request(self.freq, rf_freq=self.freq - self.lo_offset,
                                                                rf_freq_policy=uhd.tune_request.POLICY_MANUAL), 0)
        self.uhd_usrp_sink_0.set_center_freq(uhd.tune_request(self.freq, rf_freq=self.freq - self.lo_offset,
                                                              rf_freq_policy=uhd.tune_request.POLICY_MANUAL), 0)

    def get_freq(self):
        return self.freq

    def set_freq(self, freq):
        self.freq = freq
        self._freq_callback(self.freq)
        self.wifi_phy_hier_0.set_frequency(self.freq)
        self.uhd_usrp_source_0.set_center_freq(uhd.tune_request(self.freq, rf_freq=self.freq - self.lo_offset,
                                                                rf_freq_policy=uhd.tune_request.POLICY_MANUAL), 0)
        self.uhd_usrp_sink_0.set_center_freq(uhd.tune_request(self.freq, rf_freq=self.freq - self.lo_offset,
                                                              rf_freq_policy=uhd.tune_request.POLICY_MANUAL), 0)

    def get_encoding(self):
        return self.encoding

    def set_encoding(self, encoding):
        self.encoding = encoding
        self._encoding_callback(self.encoding)
        self.wifi_phy_hier_0.set_encoding(self.encoding)

    def get_chan_est(self):
        return self.chan_est

    def set_chan_est(self, chan_est):
        self.chan_est = chan_est
        self._chan_est_callback(self.chan_est)
        self.wifi_phy_hier_0.set_chan_est(self.chan_est)

    def send_pkt(self, pkt, encoding):
        """
        Send a packet from PHY to air. FCS (crc32) is not included
        :param pkt: (string) PSDU to be sent
        :param encoding: data rate [0, 8)
        :return: none
        """
        port = pmt.intern("in")  # message input port name of block mapper()
        self.set_encoding(encoding)  # select data rate

        # print "TX PKT: ", ','.join(x.encode('hex') for x in pkt)

        crc_dict = pmt.make_dict()
        crc_dict = pmt.dict_add(crc_dict, pmt.string_to_symbol("crc_included"), pmt.PMT_F)

        p = pmt.make_u8vector(len(pkt), 0)
        pkt1 = list(pkt)
        for i in range(len(pkt1)):
            pmt.u8vector_set(p, i, ord(pkt1[i]))

        self.wifi_phy_hier_0.ieee802_11_mapper_0.to_basic_block()._post(port, pmt.cons(crc_dict, p))


class rx_client(threading.Thread):
    """
    Check incoming packets received from wifi PHY (wifi_transceiver),
    and store the packets to corresponding buffers
    :param PHYRXport: socket port
    :param my_mac: local MAC address
    :param node: USRP node No. (for print use only)
    :param print_buffer: flag of printing buffer size
    :param print_beacon: flag of printing beacons
    :param short_beacon_info: True - print partial Beacon info
    :return: none
    """

    def __init__(self, PHYRXport, my_mac, node, print_buffer=False, print_beacon=False, short_beacon_info=True):
        threading.Thread.__init__(self)

        self.my_mac = my_mac
        self.node = node
        self.print_buffer = print_buffer
        self.print_beacon = print_beacon
        self.short_beacon_info = short_beacon_info

        self.context = zmq.Context()
        self.results_receiver = self.context.socket(zmq.PULL)
        self.results_receiver.connect("tcp://127.0.0.1:%d" % PHYRXport)

        self.running = True

    def run(self):
        print_msg("rx_client starts to check received packets from PHY (class wifi_transceiver)", self.node)

        while self.running:
            # PHY 802.11 frame arrival from the wireless medium
            try:
                raw_data = self.results_receiver.recv()
                pmt_pair = pmt.deserialize_str(raw_data)
                pkt_info = pmt.to_python(pmt.car(pmt_pair))  # Packet info, such as encoding, freq, freq_offset, ...
                blob_data = pmt.to_python(pmt.cdr(pmt_pair))
                pkt = "".join(chr(x) for x in blob_data)
            except:
                continue

            arrived_packet = mac.parse_mac(pkt, pkt_info)

            if "DATA" == arrived_packet["HEADER"] or "DATA_FRAG" == arrived_packet["HEADER"]:  # DATA
                if self.my_mac == arrived_packet["DATA"]["mac_add1"]:  # Is DATA addressed to this node?
                    data.put(arrived_packet["DATA"])
                else:
                    other.put("random data")
            elif "ACK" == arrived_packet["HEADER"]:  # ACK
                if self.my_mac == arrived_packet["DATA"]["RX_add"]:  # Is ACK addressed to this node?
                    ack.put(arrived_packet["DATA"])
            elif "RTS" == arrived_packet["HEADER"]:  # RTS
                rts.put(arrived_packet["DATA"])
            elif "CTS" == arrived_packet["HEADER"]:  # CTS
                cts.put(arrived_packet["DATA"])
            elif "BEACON" == arrived_packet["HEADER"]:  # BEACON
                beacon = arrived_packet["DATA"]
                msg = plcp.new_beacon()
                msg["MAC"] = beacon["mac_add2"]
                msg["SSID"] = beacon["SSID"]
                msg["timestamp"] = beacon["timestamp"]
                msg["BI"] = beacon["BI"]
                msg["OFFSET"] = time.time() - beacon["timestamp"]
                x = bcn.qsize()
                updated = False
                mac_bcn = msg["MAC"]

                # Update the beacon list
                while x > 0:
                    tmp = bcn.get()
                    if mac_bcn != tmp["MAC"]:
                        bcn.put(tmp)
                        x -= 1
                    else:  # Update the AP state
                        bcn.put(msg)
                        updated = True
                        break

                if not updated:  # AP is not in the list
                    bcn.put(msg)
            else:
                continue

            if self.print_buffer:
                # Queue size
                print_msg("=========== BUFFER STATUS ===========", self.node)
                print_msg("DATA   [%i]" % data.qsize(), self.node)
                print_msg("ACK    [%i]" % ack.qsize(), self.node)
                print_msg("RTS    [%i]" % rts.qsize(), self.node)
                print_msg("CTS    [%i]" % cts.qsize(), self.node)
                print_msg("BEACON [%i]" % bcn.qsize(), self.node)
                print_msg("OTHER  [%i]" % other.qsize(), self.node)

            if self.print_beacon:
                # Beacon list
                print_msg("===== NEIGHBOR NODES INFORMATION ====", self.node)
                x = bcn.qsize()
                if self.short_beacon_info:
                    print_msg("      MAC             SSID", self.node)
                    while x > 0:
                        item = bcn.get()
                        print_msg("%s %s" % (mac.format_mac(item["MAC"]), item["SSID"]), self.node)
                        bcn.put(item)
                        x -= 1

                else:
                    print_msg("      MAC           Timestamp   BI      OFFSET         SSID", self.node)
                    while x > 0:
                        item = bcn.get()
                        print_msg("%s %d %s %d %s" % (
                            mac.format_mac(item["MAC"]), item["timestamp"], item["BI"], item["OFFSET"], item["SSID"]),
                                  self.node)
                        bcn.put(item)
                        x -= 1
                print_msg("=====================================", self.node)

    def stop(self):
        self.running = False
        self.results_receiver.close()
        self.context.term()
        print_msg("Thread rx_clients() exits", self.node)


class proc_mac_request(threading.Thread):
    """
    Process request from MAC layer
    :param options: TX parameters
    :param msgq: msg queue that stores the power level (for CCA use)
    :param wifi_transceiver: wifi_transceiver class
    :return: none
    """

    def __init__(self, options, msgq, wifi_transceiver):
        threading.Thread.__init__(self)

        self.msgq = msgq
        self.wifi_transceiver = wifi_transceiver
        self.node = options.node
        self.samp_rate = options.samp_rate
        self.verbose = options.verbose

        # Stream sockets to ensure no packet loss during PHY<-->MAC communication
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((socket.gethostname(), options.PHYport))
        self.server.listen(1)  # PHY is ready to attend MAC requests

        self.running = True

    def run(self):
        # Initial values of variables used in time measurement
        t_socket_TOTAL = 0  # Total time of socket communication
        T_sense_USRP2 = 0  # Time of the USRP2 measuring the power
        T_sense_PHY = 0  # Time elapsed in PHY layer due to a carrier sensing request
        T_transmit_USRP2 = 0  # USRP2 TX time
        T_configure_USRP2 = 0  # Time due to USRP2 graph management
        T_transmit_PHY = 0  # Time elapsed in PHY layer due to a packet tx request

        n_ack_rx = 0  # Number of ACK received
        n_data_tx = 0  # Number of packets sent
        n_data_retx = 0  # Number of retransmitted DATA frame
        n_data_rx = 0  # Number of DATA frame received (always equal to n_ack_tx)
        n_cca = 0  # Number of power measurements (0: disable carrier sensing)

        while self.running:
            try:
                socket_client, _ = self.server.accept()  # Waiting a request from the MAC layer
            except:
                continue
            arrived_packet = plcp.receive_from_mac(socket_client)  # Packet received from MAC

            print_stat = False
            if "PKT" == arrived_packet["HEADER"]:
                print_msg("Mac requests a packet transmission", self.node, False)
                print_stat = True
                pkt_type = arrived_packet["DATA"]["HEADER"]
                if "DATA" == pkt_type:
                    n_data_tx += 1
                elif "DATA_RETX" == pkt_type:
                    n_data_retx += 1
                t_socket = time.time() - arrived_packet["DATA"]["INFO"]["timestamp"]
                t_socket_TOTAL = t_socket_TOTAL + t_socket  # Update the time used in the socket communication.

                t_sendA = time.time()
                item = arrived_packet["DATA"]["INFO"]["packet"]  # Copy the packet to send from the MAC message

                r = gr.enable_realtime_scheduling()
                print_msg("Warning: failed to enable realtime scheduling", self.node, r != gr.RT_OK)
                t_2 = time.time()  # TODO: fix the timestamps
                t_sendB = time.time()

                self.wifi_transceiver.send_pkt(item, arrived_packet["DATA"]["INFO"]["encoding"])

                t_sendC = time.time()
                t_sendD = time.time()
                print_msg("Time elapsed on graph configuration (TX Packet) = %f" % (t_sendB - t_2), self.node,
                          self.verbose)
                T_transmit_USRP2 = T_transmit_USRP2 + t_sendC - t_sendB
                T_configure_USRP2 = T_configure_USRP2 + t_sendD - t_sendA - (t_sendC - t_sendB)
                T_transmit_PHY = T_transmit_PHY + t_sendD - t_sendA

            elif "CCA" == arrived_packet["HEADER"]:  # Carrier sensing request
                print_msg("Mac requests a CCA", self.node, False)
                t_senseA = time.time()
                self.msgq.flush()
                t_reconfig = time.time() - t_senseA
                m = self.msgq.delete_head()
                t = m.to_string()
                msgdata = struct.unpack('%df' % (int(m.arg2()),), t)
                sensed_power = msgdata[0]
                t_senseB = time.time()

                packet = plcp.create_packet("CCA", sensed_power)
                plcp.send_to_mac(socket_client, packet)
                t_senseC = time.time()
                T_sense_USRP2 = T_sense_USRP2 + (t_senseB - t_senseA)
                T_sense_PHY = T_sense_PHY + (t_senseC - t_senseA)
                n_cca += 1
                print_msg("Time elapsed on graph configuration (Carrier Sensing) = %f" % t_reconfig, self.node,
                          self.verbose)

            elif "TAIL" == arrived_packet["HEADER"]:  # MAC requests an incoming packet from the PHY
                header_pkt = arrived_packet["DATA"]
                print_msg("Mac requests PHY to report a %s pkt" % header_pkt, self.node, False)

                if header_pkt == "DATA" and not data.empty():  # There are Data packets?
                    print_stat = True
                    n_data_rx += 1
                    phy_pkt = plcp.create_packet("YES", data.get())

                elif header_pkt == "ACK" and not ack.empty():  # There are ACK packets?
                    print_stat = True
                    n_ack_rx += 1
                    phy_pkt = plcp.create_packet("YES", ack.get())

                elif header_pkt == "RTS" and not rts.empty():  # There are RTS packets?
                    phy_pkt = plcp.create_packet("YES", rts.get())

                elif header_pkt == "CTS" and not cts.empty():  # There are CTS packets?
                    phy_pkt = plcp.create_packet("YES", cts.get())

                elif header_pkt == "NODE":
                    phy_pkt = plcp.create_packet("YES", self.node)

                elif header_pkt == "SAMP_RATE":
                    phy_pkt = plcp.create_packet("YES", self.samp_rate)

                else:  # There are no packets
                    phy_pkt = plcp.create_packet("NO", [])

                plcp.send_to_mac(socket_client, phy_pkt)  # Send the result (PHY packet) to MAC layer

            if self.verbose and n_cca > 0:
                print_msg("===================== Average statistics ====================", self.node)
                print_msg("No. of carrier sensing requests = %d" % n_cca, self.node)
                print_msg("Time spent by USRP2 on sensing channel = %f" % (T_sense_USRP2 / n_cca), self.node)
                print_msg("Time spent by PHY layer on sensing channel = %f" % (T_sense_PHY / n_cca), self.node)
                n_ack_rx = 0  # Number of ACK received
                n_data_tx = 0  # Number of packets sent
                n_data_retx = 0  # Number of retransmitted DATA frame
                n_data_rx = 0  # Number of DATA frame received
            print_msg("CCA: %d, DATA TX: %d, DATA RETX: %d, ACK RX: %d, DATA RX: %d" % (
                n_cca, n_data_tx, n_data_retx, n_ack_rx, n_data_rx), self.node, print_stat)
            if self.verbose and n_data_tx > 1:
                print_msg("Time spent by USRP2 on sending a packet = %f" % (T_transmit_USRP2 / n_data_tx), self.node)
                print_msg("Time spent by USRP2 on configuring the graphs = %f" % (T_configure_USRP2 / n_data_tx),
                          self.node)
                print_msg("Time spent by PHY on sending a packet = %f" % (T_transmit_PHY / n_data_tx), self.node)
                print_msg("Time spent on Socket Communication = %f" % (t_socket_TOTAL / n_data_tx), self.node)
                print_msg("=============================================================", self.node)

    def stop(self):
        self.running = False
        self.server.shutdown(socket.SHUT_RDWR)
        self.server.close()
        print_msg("Thread proc_mac_request() exits", self.node)


data = Queue()
ack = Queue()
rts = Queue()
cts = Queue()
bcn = Queue()
other = Queue()


def main(top_block_cls=wifi_transceiver, options=None):
    from distutils.version import StrictVersion
    if StrictVersion(Qt.qVersion()) >= StrictVersion("4.5.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    parser = OptionParser(option_class=eng_option, usage="%prog: [options]")
    parser.add_option("-f", "--freq", type="eng_float",
                      default=5.210e9, help="set USRP2 carrier frequency, [default=%default]", )
    parser.add_option("-o", "--lo_offset", type="eng_float", default=0,
                      help="set USRP2 lo frequency offset, [default=%default]")
    parser.add_option("-w", "--samp_rate", type="eng_float", default=20e6, help="set bandwidth [default=%default]\
                        20 MHz -> 802.11a/g, OFDM-symbol duration=4us,\
                        10 MHz -> 802.11p, OFDM-symbol duration=8us")
    parser.add_option("-a", "--usrp_ip", type="string", default="",
                      help="USRP ip address [default=%default]")
    parser.add_option("-g", "--rx_gain", type="eng_float", default=5.7,
                      help="set USRP2 Rx GAIN in [dB] [default=%default]")
    parser.add_option("-e", "--chan_est", type="int", default=ieee802_11.LS,
                      help="Equalizer algorithm [default=%default]\
                        0 -> LS, \
                        1 -> LMS, \
                        2 -> COMB, \
                        3 -> STA")
    parser.add_option("", "--encoding", type="int", default=0,
                      help="set OFDM data rate, [default=%default]\
                        0 -> 6 (3) Mbit/s (BPSK r=0.5), \
                        1 -> 9 (4.5) Mbit/s (BPSK r=0.75), \
                        2 -> 12 (6) Mbit/s (QPSK r=0.5), \
                        3 -> 18 (9) Mbit/s (QPSK r=0.75), \
                        4 -> 24 (12) Mbit/s (QAM16 r=0.5), \
                        5 -> 36 (18) Mbit/s (QAM16 r=0.75), \
                        6 -> 48 (24) Mbit/s (QAM64 r=0.66), \
                        7 -> 54 (27) Mbit/s (QAM64 r=0.75)")
    parser.add_option("-G", "--tx_gain", type="eng_float", default=23.5,
                      help="set USRP2 Tx GAIN in [dB] [default=%default]")
    parser.add_option("-n", "--node", type="int", default=1, help="USRP2 node [default=%default]")
    parser.add_option("", "--PHYRXport", type="int", default=8513,
                      help="Socket port for PHY internal RX use [default=%default]")
    parser.add_option("", "--PHYport", type="int", default=8013,
                      help="Socket port for MAC <-> PHY communication [default=%default]")
    parser.add_option('-v', "--verbose", action="store_true", default=False,
                      help="Print timing information, [default=%default]")

    (options, args) = parser.parse_args()

    usrp_ip = options.usrp_ip
    if "" != usrp_ip and not usrp_ip.startswith("addr="):  # USRP address, if specified, must start with "addr="
        options.usrp_ip = "addr=%s" % usrp_ip

    assert options.samp_rate in [10e6, 20e6], "Incorrect sample_rate. [10 or 20 MHz]"

    my_mac = mac.assign_mac(options.node)  # Assign the MAC address of the node

    print_msg("-------------------------", options.node)
    print_msg("... PHY layer running ...", options.node)
    print_msg(" (Ctrl + C) to exit", options.node)
    print_msg("Node %d - %s" % (options.node, mac.format_mac(my_mac)), options.node)
    print_msg("USRP IP: %s" % options.usrp_ip, options.node)
    print_msg("-------------------------", options.node)

    rx_client_thread = rx_client(options.PHYRXport, my_mac, options.node)
    rx_client_thread.start()

    msgq = gr.msg_queue(1)

    tb = top_block_cls(options, socket.gethostname(), msgq)
    tb.start()
    tb.show()

    proc_mac_thread = proc_mac_request(options, msgq, tb)
    proc_mac_thread.start()

    def quitting():
        tb.stop()
        tb.wait()
        rx_client_thread.stop()
        proc_mac_thread.stop()

    qapp.connect(qapp, Qt.SIGNAL("aboutToQuit()"), quitting)
    qapp.exec_()


if __name__ == '__main__':
    main()
