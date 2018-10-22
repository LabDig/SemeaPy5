sim800l = serial.Serial ("/dev/ttyS1", 4800) # P9_24 P9_26
sim800l.write('AT+SAPBR=3,1,\"Contype\",\"GPRS\"'+'\r\n');
sim800l.write('AT+SAPBR=3,1,\"APN\",\"zap.vivo.com.br\"'+'\r\n');
sendAtCmd('AT+SAPBR=1,1'+'\r\n');
sim800l.write('AT+SAPBR=2,1'+'\r\n');
sim800l.write('AT+HTTPINIT'+'\r\n');
sim800l.write('AT+HTTPPARA=\"CID\",1'+'\r\n');
#sim800l.write('AT+HTTPPARA=\"URL\",\"http://ecosolucoes.net/andre/salvaarduino.php?ph=0.0\"'+'\r\n');
#sim800l.write('AT+HTTPACTION=0'+'\r\n');
#sim800l.write('AT+HTTPREAD'+'\r\n');
