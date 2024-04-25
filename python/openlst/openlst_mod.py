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

class openlst_mod(gr.basic_block):
	"""
	OpenLST Encoder/Framer

	This block encodes a raw data packet in the form:
	
		| Sequence bytes (2 bytes) | Length (1 byte) | HWID (2 bytes) | Seqnum (2 bytes) | Data (N bytes) |
	
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
			whitening=True,
			seq_byte1=0x22,
			seq_byte0=0x69
		):
		gr.basic_block.__init__(
			self,
			name="OpenLST Encode and Frame",
			in_sig=[np.uint8],
			out_sig=[np.uint8]
		)

		self.preamble_bytes = preamble_bytes
		self.sync_byte1 = sync_byte1
		self.sync_byte0 = sync_byte0
		self.sync_words = sync_words
		self.flags = flags
		self.fec = fec
		self.whitening = whitening
		self.sequence_bytes = [seq_byte1, seq_byte0]

		self._in_buffer = []
		self._out_buffer = []

		self._input_items_offset = 0
		self._to_consume = 0

		self._mode = 'sequence-bytes'
		self._length = 0

		self._output_multiple = 16
		self.set_output_multiple(self._output_multiple)

	def forecast(self, noutput_items, ninputs):
		return [int(noutput_items/self._output_multiple) for i in range(ninputs)]
	
	def consume_input(self, num, pop_buffer=True, call_consume=True):
		if pop_buffer:
			self._in_buffer = self._in_buffer[num:]
		if call_consume:
			self.consume(0, num)
			self._to_consume = max(0, self._to_consume - num)
			self._input_items_offset = max(0, self._input_items_offset - num)
		else:
			self._to_consume += num

	def general_work(self, input_items, output_items):
		self._in_buffer.extend(input_items[0][self._input_items_offset:])
		self._input_items_offset += len(input_items[0][self._input_items_offset:])

		if self._mode == 'sequence-bytes':
			while len(self._in_buffer) >= len(self.sequence_bytes):
				sb = self._in_buffer[:2]
				if sb == self.sequence_bytes:
					print("[Mod] Sequence bytes matched")
					self._mode = 'length'
					self.consume_input(2)
					break
				else:
					self.consume_input(1)
		if self._mode == 'length':
			if len(self._in_buffer) >= 1:
				self._length = self._in_buffer[0]
				print("[Mod] Length matched:", self._length)
				self._mode = 'data'
				self.consume_input(1)
		if self._mode == 'data':
			if len(self._in_buffer) >= self._length:
				raw = bytes(self._in_buffer[:self._length])

				# Insert the preamble and sync words
				preamble = bytes(
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
				encoded_data = [byte for byte in preamble + content + bytes([0]*8)]
				# encoded_data = [int(bit) for byte in preamble + content for bit in bin(byte)[2:].zfill(8)]
				self._out_buffer.extend(encoded_data)

				self.consume_input(self._length, call_consume=False)
				self._length = 0
				self._mode = "sequence-bytes"
			else:
				self.consume_input(len(input_items[0]), pop_buffer=False)

		if len(self._out_buffer) > 0:
			# Get sizes of out_buffer and output_items
			buffer_size = len(self._out_buffer)
			output_size = len(output_items[0])

			# Get number of bytes out as the min of those two
			num_bytes_out = min(buffer_size, output_size)

			# Split the buffer into what we're sending out and the remaining buffer
			data = self._out_buffer[:num_bytes_out]
			buff = self._out_buffer[num_bytes_out:]

			# data.extend(0)

			# Add data to output
			output_items[0][:num_bytes_out] = data
			print(f"[Mod] Data out: {data}")

			# Set out_buffer to new buffer
			self._out_buffer = buff

			# If at the end of sending the packet, consume all of the input for it
			if (buffer_size <= output_size):
				self.consume_input(self._to_consume, pop_buffer=False)

			if num_bytes_out == 32:
				pass

			# Return number of bytes out
			print(num_bytes_out)
			return num_bytes_out
		
		# Return 0 for no bytes to return
		return 0