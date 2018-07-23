import math

def Fert(v,rate,spacing):
    fertbym=rate*spacing/10000.0
    factor=0.026  #kg of fertilizer in one revolution of screw
    return 60*fertbym*v/factor,fertbym

def Seeder(v,pop,row,holes,germ):
    seeds=pop*row/(10000*germ/100)
    return 60*3.3*seeds*v/holes,seeds

# Find the Neart point in the map
def FindNeig(x_atual,y_atual,x_map,y_map,pop_map):
    minDist=9999999
    k=len(x_map)
    for i in range(k):
        deltaX=math.fabs(x_atual-x_map[i])
        deltaY=math.fabs(y_atual-y_map[i])
        distance=math.sqrt(deltaX*deltaX+deltaY*deltaY)
        if distance<minDist:
            minDist=distance
            idminDist=i
            
    return pop_map[idminDist],x_map[idminDist],y_map[idminDist]
