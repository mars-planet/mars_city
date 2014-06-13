import sys
from pocketsphinx import Decoder
import pyaudio
import wave
import os
import audioop

#run importacoustic.sh
hmdir = "~/voicelib/hub4wsj_sc_8kadapt"
lmdir = "5356.lm"
dictd = "5356.dic"
sysdir = os.getcwd

def getScore(data):
	rms = audioop.rms(data, 2)
	score = rms / 3
	return score


def fetchThreshold():

	RATE = 16000
	CHUNK = 1024
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


def passiveListen():


	THRESHOLD_MULTIPLIER = 1.8
	AUDIO_FILE = "passive.wav"
	RATE = 16000
	CHUNK = 1024

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
	lastN = [i for i in range(30)]

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

	# save some memory for sound data
	frames = []

	# flag raised when sound disturbance detected
	didDetect = False

	# start passively listening for disturbance above threshold
	for i in range(0, RATE / CHUNK * LISTEN_TIME):

		data = stream.read(CHUNK)
		frames.append(data)
		score = getScore(data)

		if score > THRESHOLD:
			didDetect = True
			break

	# no use continuing if no flag raised
	if not didDetect:
		print "No disturbance detected"
		return

	# cutoff any recording before this disturbance was detected
	frames = frames[-20:]

	# otherwise, let's keep recording for few seconds and save the file
	DELAY_MULTIPLIER = 1
	for i in range(0, RATE / CHUNK * DELAY_MULTIPLIER):

		data = stream.read(CHUNK)
		frames.append(data)

	# save the audio data
	stream.stop_stream()
	stream.close()
	audio.terminate()
	write_frames = wave.open(AUDIO_FILE, 'wb')
	write_frames.setnchannels(1)
	write_frames.setsampwidth(audio.get_sample_size(pyaudio.paInt16))
	write_frames.setframerate(RATE)
	write_frames.writeframes(''.join(frames))
	write_frames.close()
	passivewav= sysdir+"/passive.wav"
	# check if PERSONA was said
	speechRec = Decoder(hmm = hmdir, lm = lmdir, dict = dictd)
	with open(passivewav, 'rb') as passivewav:
        speechRec.decode_raw(passivewav)
        result = speechRec.get_hyp()

	if "trevor" in result[0]:
		return (THRESHOLD, result[0])

	return (False,result[0])


def record(THRESHOLD=None):

    chunk = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 16000
    RECORD_SECONDS = 2
    WAVE_OUTPUT_FILENAME = "livewav.wav"

    p = pyaudio.PyAudio()

    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=chunk)

	if THRESHOLD == None:
		THRESHOLD = fetchThreshold()


    print "* recording"
    frames = []
    #for i in range(0, RATE / chunk * RECORD_SECONDS):
        #data = stream.read(chunk)
        ##stores data into list "frames"
        #frames.append(data)

   #plays low beep
    lastN = [THRESHOLD * 1.2 for i in range(30)]

        for i in range(0, RATE / CHUNK * LISTEN_TIME):

            data = stream.read(CHUNK)
            frames.append(data)
            score = self.getScore(data)

            lastN.pop(0)
            lastN.append(score)

            average = sum(lastN) / float(len(lastN))

            # TODO: 0.8 should not be a MAGIC NUMBER!
            if average < THRESHOLD * 0.8:
                break
    print "* done recording"
    stream.stop_stream()
    stream.close()
    p.terminate()

    # write data to WAVE file
    data = ''.join(frames)
    wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(data)
    wf.close()
    sysdir = os.getcwd()
    wavfile = sysdir+"/livewav.wav"


    speechRec = Decoder(hmm=hmdir, lm=lmdir, dict=dictd)
    with open(wavfile, 'rb') as wavFile:
        speechRec.decode_raw(wavFile)
        result = speechRec.get_hyp()

    return(result[0])

    #DO SOME AMPLIFICATION
    #os.system("sox "+WAVE_OUTPUT_FILENAME+" temp.wav vol 15dB")

def handleForever():
        """Delegates user input to the handling function when activated."""
        while True:
            try:
                threshold, transcribed = passiveListen()
            except:
                continue

            if threshold:
				#input=record()
                activerec = record(threshold)
                #if input:
                    #self.delegateInput(input)
                #else:
                    #self.mic.say("Pardon?")
	return(activerec)
if __name__ == "__main__":

    #hmdir = "/usr/share/pocketsphinx/model/hmm/en_US/hub4wsj_sc_8k"
    #lmdir = "/usr/share/pocketsphinx/model/lm/en_US/hub4.5000.DMP"
    #dictd = "/usr/share/pocketsphinx/model/lm/en_US/cmu07a.dic"


    os.system("aplay beep_hi.wav")
    #record()
    recognised=handleForever()
    os.system("aplay beep_lo.wav")

    #wavFile = file(wavfile , 'rb')
    #speechRec.decode_raw(wavFile)
    ###print speechRec
    #result = speechRec.get_hyp()
    #passiveListen()

print "Recognised text from recorded file"

print "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
print recognised[0]
print "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
