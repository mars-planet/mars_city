#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  speechrecognition(pep8).py
#
#  Copyright 2014 Shridhar Mishra <shridhar@shridhar>
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


def record():

    chunk = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 44100
    RECORD_SECONDS = 2
    WAVE_OUTPUT_FILENAME = "livewav.wav"

    p = pyaudio.PyAudio()

    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=chunk)

    print "* recording"

    #plays beep

all = []
for i in range(0, RATE / chunk * RECORD_SECONDS):
    data = stream.read(chunk)
    all.append(data)

#plays low beep
    print "* done recording"

    stream.close()
    p.terminate()

    # write data to WAVE file
    data = ''.join(all)
    wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(data)
    wf.close()

    #DO SOME AMPLIFICATION
    #os.system("sox "+WAVE_OUTPUT_FILENAME+" temp.wav vol 15dB")


if __name__ == "__main__":

    #hmdir = "/usr/share/pocketsphinx/model/hmm/en_US/hub4wsj_sc_8k"
    #lmdir = "/usr/share/pocketsphinx/model/lm/en_US/hub4.5000.DMP"
    #dictd = "/usr/share/pocketsphinx/model/lm/en_US/cmu07a.dic"

    hmdir = "/usr/share/pocketsphinx/model/hmm/en_US/hub4wsj_sc_8k"
    lmdir = "1609.lm"
    dictd = "1609.dic"
    os.system("aplay beep_hi.wav")
    record()
    os.system("aplay beep_lo.wav")
    wavfile = "/home/shridhar/gsoc/pocketsphinxtest/livewav.wav"

    speechRec = Decoder(hmm=hmdir, lm=lmdir, dict=dictd)
    wavFile = file(wavfile, 'rb')
    speechRec.decode_raw(wavFile)
    ##print speechRec
    result = speechRec.get_hyp()

    ##print "popat"
print "Recognised text from the converted video file"

print "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
print result[0]
print "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
