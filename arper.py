
import sys
import time
from rtmidi.midiutil import open_midiport
import numpy as np
from rtmidi.midiconstants import *
import copy


class Handler(object):
    def __init__ (self, port, Arper):
        self.port = port
        self.nChannels = 20
        self.notes = np.zeros(self.nChannels)
        self.noteOnOff = np.zeros(self.nChannels)
        self.X = np.zeros(self.nChannels) # up down
        self.Y = np.zeros(self.nChannels) # left right
        self.Z = np.zeros(self.nChannels) # press
        self.Arper = Arper

    def __call__ (self, event, data=None):
        message = event
        mdata = message[0]
        channel  = mdata[0] & 0b00001111
        miditype = mdata[0] & 0b11110000
        # midi types: http://www.music.mcgill.ca/~ich/classes/mumt306/StandardMIDIfileformat.html
        if miditype == 0b1000 << 4:
            # print("note of")
            note = mdata[1]
            vel = mdata[2]
            self.notes[channel] = note
            self.noteOnOff[channel] = 0

        if miditype == 0b1001 << 4:
            # print("note on")
            note = mdata[1]
            vel = mdata[2]
            self.notes[channel] = note
            self.noteOnOff[channel] = 1

        if miditype == 0b1101 << 4:
            # print("channel aftertouch push down onto")
            self.Z[channel] = mdata[1]

        if miditype == 0b1011 << 4 and mdata[1] == 74:
            # print("cc slide up and down")
            self.X[channel] = mdata[2]

        if miditype == 0b1110 << 4:
            # print("pitch left right")
            self.Y[channel] = (mdata[1] + 128* mdata[2])
            # print(mdata[2])
        # print(channel, miditype)
        self.Arper.update_seq(self.get_keypresses())
    
    def get_keypresses(self):
        indices = np.argsort(self.notes)
        
        keys = [{"note":self.notes[idx], "x":self.X[idx], "y":self.Y[idx], "z":self.Z[idx]} for idx in indices if self.noteOnOff[idx]]
         
        return keys
        


class Arper:
    def __init__(self, dt, t, seq_fun=None, pattern=None):
        self.base_seq = []     # sequence obtained from keypresses 
        self.base_seq_len = 0  #
        self.seq = []          # sequence created based on the base seq, defined by seq_fun (if provided)
        self.seq_len = 0
        self.seq_fun = seq_fun

        #initialization
        self.dt = dt
        self.init_time = t
        self.last_step = -1
        self.step = 0

    def update_seq(self, mpe_seq):
        self.base_seq = mpe_seq # base sequence from mpe device
        self.base_seq_len = len(self.base_seq) # length of the base seq
        if self.seq_fun is not None and self.base_seq_len > 0: # if the sequence should be modified, appl yuser supplied filter funct
            self.seq = self.seq_fun(self.base_seq)
        else:
            self.seq = self.base_seq
        self.seq_len = len(self.seq)

    def print_info(self):
        print(self.base_seq)


    def handle_step(self, t):
        rel_time = t-self.init_time
        self.step = np.floor(rel_time/self.dt) 
        
        # spit out a not, if ready.
        if self.step > self.last_step: # if dt has passed
            rel_step = self.step%self.seq_len # relative step
            note = self.get_note(rel_step) # get note associated to this step
            self.last_step = self.step # update
            return note

        else: 
            return None

    def get_note(self,rel_step):
        if not np.isnan(rel_step):
            return (self.base_seq[int(rel_step)]) # TODO: include different patterns here
        else:
            return None

def transpose(note,octave):
    new_note = copy.copy(note)
    new_note["note"] += octave*12
    return new_note

# test the sequence custom function
def test_seq_func(seq):
    seq.append(transpose(seq[-1],2))
    seq.append(transpose(seq[0],-1))
    return seq




# midi io
midiin,    in_port_name  = open_midiport()
midiout,  out_port_name  = open_midiport(type_="output")
# arper object
arp = Arper(0.2,time.time(),seq_fun=test_seq_func)
# midi handler
handler = Handler(in_port_name, arp)
midiin.set_callback(handler)

# mapping 1
try:
    while True:
        time.sleep(0.001)
        # arp.print_info()
        note = arp.handle_step(time.time())
        if note is not None:
            print(note["note"])
            print( note["x"], note["y"])
            vel = note["z"] # z is mapped to note on velocity
            pitchbend = int(note["y"]) # TODO set this in driver maybe ?
            octave = np.floor(note["x"]/25)-2
            midiout.send_message([NOTE_ON,note["note"]+12*octave,vel])
            midiout.send_message([PITCH_BEND, pitchbend & 0x7f, (pitchbend >> 7) & 0x7f])
            time.sleep(0.1)
            midiout.send_message([NOTE_OFF,note["note"]+12*octave,0])


        # print(handler.get_keypresses())
        

except KeyboardInterrupt:
    print (" ")

finally:
    print ("Exit..")
    midiin.close_port()
    del midiin
