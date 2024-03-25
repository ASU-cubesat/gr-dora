#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2023 Robert Zimmerman.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

import numpy as np
from gnuradio import gr

from .fec import encode_fec
from .whitening import whiten
from .crc import crc16

class openlst_mod(gr.sync_block):
	"""
	OpenLST Encoder/Framer

	This block encodes a raw data packet in the form:
	
		| HWID (2 bytes) | Seqnum (2 bytes) | Data (N bytes) |

	aCtualLy it's THIS:
		| Sequence bytes (2 bytes) | Length (1 byte) | HWID (2 bytes) |
			Seqnum (2 bytes) | System byte (1 byte)  | Data (N bytes) |
	
	To an RF message:

		| Preamble | Sync Word(s) | Data Segment |
	
	Where "Data Segment" contains:

		| Length (1 byte) | Flags (1 byte) | Seqnum (2 bytes) | Data (N bytes) | HWID (2 bytes) | CRC (2 bytes)
	
	And may be encoded with whitening (PN-9 coding) and/or 2:1 Forward-Error Correction (FEC).

	It supports throttling of the output data rate for low bitrates. This avoids filling up the
	(very large) buffer of the downstream blocks and inducing a lot of latency.
	"""
	def __init__(
			self,
			preamble_bytes=4,
			sync_byte1=0xd3,
			sync_byte0=0x91,
			sync_words=2,
			flags=0xC0,
			fec=True,
			whitening=True
		):
		gr.sync_block.__init__(
			self,
			name="OpenLST Encode and Frame",
			in_sig=[np.uint8],
			out_sig=[np.uint8],
		)

		self.preamble_bytes = preamble_bytes
		self.sync_byte1 = sync_byte1
		self.sync_byte0 = sync_byte0
		self.sync_words = sync_words
		self.flags = flags
		self.fec = fec
		self.whitening = whitening

		self._buff = []
		self._out_buffer = []

	def handle_msg(self, msg):
		raw = bytearray(msg)

		# Insert the preamble and sync words
		preamble = bytearray(
			[0xaa] * self.preamble_bytes +  # preamble
			[self.sync_byte1, self.sync_byte0] * self.sync_words)  # sync word(s)
		
		# Prefix with length byte and flags
		content = bytes(
			[len(raw) + 3] +  # length = raw + flags + checksum (2 bytes)
			[self.flags]  # flags
		)
		content += raw[2:]  # data (includes seqnum)
		# The HWID goes at the end for RF transmission
		content += raw[0:2]
		checksum = crc16(content)

		# Append checksum
		content += checksum.to_bytes(2, byteorder='little')

		# Per the datasheet, whitening happens _before_ FEC
		if self.whitening:
			content = whiten(content)
		if self.fec:
			content = encode_fec(content)

		# Queue these bytes for transmission
		self._out_buffer.append(preamble + content)


	def work(self, input_items, output_items):
		# self._buff.extend(input_items[0])

		# Need a sample packet of data coming in, not sure if buffer is necessary or not.
		# Main concern is knowing when previous command stops and the next one starts.


		if len(self._out_buffer) > 0:
			# Get sizes of out_buffer and output_items
			buffer_size = len(self._out_buffer)
			output_size = len(output_items[0])

			# Get number of bytes out as the min of those two
			num_bytes_out = min(buffer_size, output_size)

			# Split the buffer into what we're sending out and the remaining buffer
			data = self._out_buffer[:num_bytes_out]
			buff = self._out_buffer[num_bytes_out:]

			# Add data to output
			output_items[0][:num_bytes_out] = data
			print(f"Data out: {data}")

			# Set out_buffer to new buffer
			self._out_buffer = buff

			# Consume unused bytes in the case that the number of output_items exceeds the data we have to send
			if output_size > num_bytes_out:
				self.consume(0, output_size - num_bytes_out)

			# Return number of bytes out
			return num_bytes_out
		
		# Consume all input items and return 0 for no bytes to return
		self.consume(0, len(input_items[0]))
		return 0
