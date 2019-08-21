import KBHit
import time

class DutyInput:
    def __init__(self):
        self.dlon = 0.
        self.dlat = 0.
        self.increment = 10.
        self.decay_lon = 1.
        self.decay_lat = 1.
    def feed(self, c):
        if c == 'w':
            self.dlon += self.increment
        elif c == 's':
            self.dlon -= self.increment
        elif c == 'a':
            self.dlat += self.increment
        elif c == 'd':
            self.dlat -= self.increment
        elif c == 'q':
            self.reset()
        else:
            pass
        self.dlon = max(-90,min(90,self.dlon))
        self.dlat = max(-70,min(70,self.dlat))
    def damp(self):
        self.dlon = self.dlon * self.decay_lon
        self.dlat = self.dlat * self.decay_lat
        #pass
    def get(self):
        dl = int(max(-100,min(100,self.dlon - self.dlat)))
        dr = int(max(-100,min(100,self.dlon + self.dlat)))
        return (dl,dr)
    def reset(self):
        self.dlon = 0
        self.dlat = 0

if __name__ == "__main__":

    kb = KBHit.KBHit()
    di = DutyInput()
    
    print('Hit any key, or ESC to exit')

    dl,dr = 0,0
    while True:
        if kb.kbhit():
            c = kb.getch()
            if ord(c) == 27: # ESC
                break
            di.feed(c)
        di.damp()
        dl,dr = di.get()
        print(dl,dr)
        
        time.sleep(0.01)
    di.reset()
    dl,dr = di.get()
    kb.set_normal_term()
