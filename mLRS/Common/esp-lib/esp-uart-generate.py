import re
import datetime


templatefname = "esp-uart-template.h"
    
F = open(templatefname, "r")
templatecode = F.read()
F.close()

def generate(code, periphname):
    outfname = "esp-"+periphname+".h"
    print('generate', outfname)
    code1 = code.replace('UART$',periphname.upper())
    code2 = code1.replace('uart$',periphname)
    code3 = code2.replace('CONV$','0')
    F = open(outfname, "w")
    F.write(code3)
    F.close() 

generate(templatecode, 'uart')
generate(templatecode, 'uartb')
generate(templatecode, 'uartc')
#generate(templatecode, 'uartd')
#generate(templatecode, 'uarte')
generate(templatecode, 'uartf')
