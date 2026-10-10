#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2011-2013 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
""" CRTP UDP Driver. Work either with the UDP server or with an UDP device
See udpserver.py for the protocol"""
import os
import socket
import threading
import binascii
from urllib.parse import urlparse

from .crtpdriver import CRTPDriver
from .crtpstack import CRTPPacket
from .exceptions import WrongUriType

__author__ = 'Bitcraze AB'
__all__ = ['UdpDriver']


def configured_uris():
    """Explicit fleet endpoints, in selection order; no AP-only discovery."""
    values = [os.environ.get('DRONE_URI', '')]
    values.extend(os.environ.get('DRONE_URIS', '').split(','))
    uris = []
    for value in values:
        uri = value.strip()
        if not uri:
            continue
        parsed = urlparse(uri)
        if (parsed.scheme != 'udp' or not parsed.hostname or
                parsed.port is None or not 1 <= parsed.port <= 65535 or
                parsed.username or parsed.password or parsed.path or
                parsed.query or parsed.fragment):
            raise ValueError('Expected udp://HOST:PORT, got {!r}'.format(uri))
        if uri not in uris:
            uris.append(uri)
    return uris


class UdpDriver(CRTPDriver):

    def __init__(self):
        CRTPDriver.__init__(self)
        self.debug = False
        self.link_error_callback = None
        self.link_quality_callback = None
        self.needs_resending = True
        self.socket = None
        # ESP-Drone compiles out its CRTP memory service.
        self.supports_memory_enumeration = False
        self._keepalive_stop = threading.Event()
        self._keepalive_thread = None

    def connect(self, uri, linkQualityCallback, linkErrorCallback):
        if not uri.startswith('udp://'):
            raise WrongUriType('Not an UDP URI')

        parse = urlparse(uri)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (parse.hostname, parse.port)
        # Let the OS allocate a source port for each drone connection.
        self.socket.bind(('', 0))
        self.socket.connect(self.addr)

        self.socket.sendto(b'\xff\x01\x01\x01', self.addr)
        self._keepalive_stop.clear()
        self._keepalive_thread = threading.Thread(target=self._keepalive, daemon=True)
        self._keepalive_thread.start()

        if self.debug:
            print("Connected to UDP server")
            print("Server is: %s:%d" % self.addr)

    def _keepalive(self):
        # Link-service sink (port 15, channel 2), followed by its checksum.
        # ESP-Drone's 1-second Wi-Fi watchdog otherwise stops idle telemetry.
        # This is not a commander setpoint and does not feed its flight watchdog.
        while not self._keepalive_stop.wait(0.25):
            try:
                self.socket.sendto(b'\xfe\xfe', self.addr)
            except OSError:
                return

    def receive_packet(self, time=0):
        if self.socket is None:
            return None
        try:
            self.socket.settimeout(time if time >= 0 else None)
            data, addr = self.socket.recvfrom(1024)
        except OSError:
            if self.debug:
                print("Socket error: socket might be closed.")
            return None

        if len(data) >= 2:
            # take the final byte as the checksum
            cksum_recv = data[len(data)-1]
            # remove the checksum from the data
            data = data[0:(len(data)-1)]
            # calculate checksum and check it with the last byte
            cksum = 0
            for i in data[0:]:
                cksum += i
            cksum %= 256
            if cksum != cksum_recv:
                if self.debug:
                    print("Checksum error {} != {}".format(cksum, cksum_recv))
                return None
            pk = CRTPPacket(data[0], list(data[1:]))
            # print the raw date
            if self.debug:
                print("recv: {}".format(binascii.hexlify(bytearray(data))))
            return pk

        else:
            return None

    def send_packet(self, pk):
        raw = (pk.header,) + pk.datat
        cksum = 0
        for i in raw:
            cksum += i
        cksum %= 256
        raw = raw + (cksum,)
        # change the tuple to bytes
        raw = bytearray(raw)
        self.socket.sendto(raw, self.addr)
        # print the raw date
        if self.debug:
            print("send: {}".format(binascii.hexlify(raw)))

    def close(self):
        # Remove this from the server clients list
        self._keepalive_stop.set()
        if self._keepalive_thread is not None:
            self._keepalive_thread.join(timeout=0.5)
        if self.socket is None:
            return
        try:
            self.socket.sendto(b'\xff\x01\x02\x02', self.addr)
        except OSError:
            pass
        if self.debug:
            print("Disconnected from UDP server")
            print("Server is: %s:%d" % self.addr)
        self.socket.close()

    def get_name(self):
        return 'udp'

    def scan_interface(self, address):
        return [[uri, ''] for uri in configured_uris()]
