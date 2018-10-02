from Adafruit_BBIO.Encoder import RotaryEncoder,eQEP0,eQEP2 # 0 == Seed # 2 == Roda 
EncRoda=RotaryEncoder(eQEP2)
EncSeed=RotaryEncoder(eQEP0)
EncRoda.frequency=1000
EncSeed.frequency=1000
