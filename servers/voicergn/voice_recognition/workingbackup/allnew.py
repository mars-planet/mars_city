#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  all new.py
#
#  Copyright 2014 Shridhar Mishra <shridharmishra4@gmail.com>
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#
#
import sys
from pocketsphinx import Decoder
import pyaudio
import wave
import os
import audioop

RATE = 16000
CHUNK = 1024

#run importacoustic.sh
#hmdir = "~/voicelib/hub4wsj_sc_8kadapt"
hmdir = "hub4wsj_sc_8kadapt"
lmdir = "6491.lm"
dictd = "a.dic"
sysdir = os.getcwd

def getScore(data):
	rms = audioop.rms(data, 2)
	score = rms / 3
	return score


def fetchThreshold():


	THRESHOLD_MULTIPLIER = 1.8
	AUDIO_FILE = "passive.wav"


	# number of seconds to allow to establish threshold
	THRESHOLD_TIME = 1

	# number of seconds to listen before forcing restart
	LISTEN_TIME = 10

	# prepare recording stream
	audio = pyaudio.PyAudio()
	stream = audio.open(format=pyaudio.paInt16,
						channels=1,
						rate=RATE,
						input=True,
						frames_per_buffer=CHUNK)

	# stores the audio data
	frames = []

	# stores the lastN score values
	lastN = [i for i in range(20)]

	# calculate the long run average, and thereby the proper threshold
	for i in range(0, RATE / CHUNK * THRESHOLD_TIME):

		data = stream.read(CHUNK)
		frames.append(data)

		# save this data point as a score
		lastN.pop(0)
		lastN.append(getScore(data))
		average = sum(lastN) / len(lastN)

	# this will be the benchmark to cause a disturbance over!
	THRESHOLD = average * THRESHOLD_MULTIPLIER

	return THRESHOLD

def decodepassive():
	speechRec = Decoder(hmm = hmdir, lm = lmdir, dict = dictd)
	with open(passivewav, 'rb') as passivewav:
		speechRec.decode_raw(passivewav)
        result = speechRec.get_hyp()
        return(result[0])
	#print "-----------------------"
	#print result[0]
	#print "-----------------------"

def record(THRESHOLD=None):


    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    LISTEN_TIME = 4
    WAVE_OUTPUT_FILENAME = "livewav.wav"

    p = pyaudio.PyAudio()
    if THRESHOLD == None:
		THRESHOLD = fetchThreshold()
		print THRESHOLD


    stream = p.open(format=FORMAT,
                    channels=1,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK)



    print "* recording"
    frames = []
    lastN = [THRESHOLD * 1.2 for i in range(30)]
    for i in range(0, RATE / CHUNK * LISTEN_TIME):
		data = stream.read(CHUNK)
		frames.append(data)
		score = getScore(data)
		lastN.pop(0)
		lastN.append(score)
		average = sum(lastN) / float(len(lastN))
		#print average,THRESHOLD * 0.8
		if average < THRESHOLD * 0.8:
			break



    print "* done recording"
    #stream.stop_stream()
    stream.close()
    p.terminate()

    # write data to WAVE file
    data = ''.join(frames)
    wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
    wf.setnchannels(1)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(data)
    wf.close()
    sysdir = os.getcwd()
    wavfile = sysdir+"/livewav.wav"
    #decoded=decodepassive()


    speechRec = Decoder(hmm=hmdir, lm=lmdir, dict=dictd)
    with open(wavfile, 'rb') as wavFile:
        speechRec.decode_raw(wavFile)
        result = speechRec.get_hyp()


    return(result[0])

    #DO SOME AMPLIFICATION
    #os.system("sox "+WAVE_OUTPUT_FILENAME+" temp.wav vol 15dB")


def main():
	a=record()
	print a
	print "done"

	return 0

if __name__ == '__main__':
	main()

