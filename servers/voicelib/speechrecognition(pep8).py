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
    frames = []
    for i in range(0, RATE / chunk * RECORD_SECONDS):
        data = stream.read(chunk)
        #stores data into list "frames"
        frames.append(data)

   #plays low beep
    print "* done recording"
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
    sysdir = os.getcwd()
    wavfile = sysdir+"/livewav.wav"

    speechRec = Decoder(hmm=hmdir, lm=lmdir, dict=dictd)
    with file(wavfile, 'rb') as wavFile:
        speechRec.decode_raw(wavFile)
        result = speechRec.get_hyp()
    #wavFile = file(wavfile , 'rb')
    #speechRec.decode_raw(wavFile)
    ###print speechRec
    #result = speechRec.get_hyp()

print "Recognised text from recorded file"

print "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
print result[0]
print "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
