"""
works with bleading edge sphinxbase and pocketsphinx only

setup instructions

mkdir voicergn;cd voicergn

git clone git://github.com/cmusphinx/sphinxbase.git
cd sphinxbase
bash autogen.sh
make
sudo make install

git clone git://github.com/cmusphinx/pocketsphinx.git
cd ..
cd pocketsphinx
bash autogen.sh
make
sudo make install
"""

from PyTango import *
import sys
from pocketsphinx import Decoder
import pyaudio
import wave
import os
import audioop
from espeak import espeak


#import PyTango

RATE = 16000
CHUNK = 1024
CHANNELS = 1
FORMAT = pyaudio.paInt16
hmdir = "hub4wsj_sc_8kadapt"
lmdir = "6491.lm"
dictd = "a.dic"
sysdir = os.getcwd
commands=["LEFT","RIGHT","FORWARD","BACKWARD","NOT"]
myro= DeviceProxy("c3/rovers/myro")
#myro.move([1.0,-1.0])
def decide(string):
	for item in commands:
		if item in string:
			print item

			if item == "LEFT":
				myro.move([1.0,-1.0])
			if item == "RIGHT":
				myro.move([1.0,1.0])
			if item == "FORWARD":
				myro.move([1.0,0.0])
			if item == "BACKWARD":
				myro.move([-1.0,0.0])
			if item == "NOT":
				myro.move([0.0,0.0])

def getScore(data):
	rms = audioop.rms(data, 2)
	score = rms / 3
	return score


def fetchThreshold():

	THRESHOLD_MULTIPLIER = 1.8
	# number of seconds to allow to establish threshold
	THRESHOLD_TIME = 1
	# number of seconds to listen before forcing restart
	LISTEN_TIME = 10
	# prepare recording stream
	audio = pyaudio.PyAudio()
	stream = audio.open(format=FORMAT, channels=CHANNELS,
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

def say(string):
	espeak.synth(string)


def record(listen_time):

    THRESHOLD=None
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
    detected=False
    for i in range(0, RATE / CHUNK * listen_time):
		data = stream.read(CHUNK)
		frames.append(data)
		score = getScore(data)
		if score < THRESHOLD:
			continue
                else:
                        detected=True
    if not detected:
        print "nothing detected"
        return("")

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
    config = Decoder.default_config()
    config.set_string('-hmm', hmdir)
    config.set_string('-lm', lmdir)
    config.set_string('-dict', dictd)
    config.set_string('-logfn', '/dev/null')

    speechRec = Decoder(config)


    with open(wavfile, 'rb') as wavFile:
        speechRec.decode_raw(wavFile)
        #result = speechRec.get_hyp()


    return(speechRec.hyp().hypstr)

while True:
	os.system("aplay beep_lo.wav")
	keyword = record(3)
        if keyword == "":
            continue
	print keyword
	if "TREVOR" in keyword:
		print "recognised"
		say("recognised")
	        os.system("aplay beep_hi.wav")
		command=record(3)
                if command == "":
                    continue
		print command
		say(command)
		decide(command)







