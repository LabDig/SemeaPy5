import math
def Seeder(v,populacao,espacamento,furos,germinacao):
    sementes=populacao*espacamento/(10000*germinacao/100)
    rotacao=60*3.3*sementes*v/furos
    return rotacao,sementes

def FindNeig(x,y,populacao,espacamento,lat,long):
    minDist=9999999
    k=len(x)
    for i in range(k):
        deltaX=math.fabs(lat-x[i])
        deltaY=math.fabs(long-y[i])
        distance=math.sqrt(deltaX*deltaX+deltaY*deltaY)
        if distance<minDist:
            minDist=distance
            idminDist=i
            
    return populacao[idminDist],espacamento[idminDist],x[idminDist],y[idminDist]
