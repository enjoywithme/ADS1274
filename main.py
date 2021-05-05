# main.py
# import network
# import webrepl
# import utime

# SSID = "dlhome-2.4G"
# PASSWORD = "18602824101000"

# def do_connect():
    # import network
    # wlan = network.WLAN(network.STA_IF)
    # wlan.active(True)
    # if not wlan.isconnected():
        # print('connecting to network...')
        # wlan.connect(SSID, PASSWORD)
        
    # start = utime.time()
    # while not wlan.isconnected():
        # utime.sleep(1)
        # if utime.time()-start > 5:
            # print("connect timeout!")
            # break
            
    # if wlan.isconnected():
        # print('network config:', wlan.ifconfig())

# do_connect()

import utime
import machine
import socket
import time
import network

from machine import Pin, SPI

from machine import WDT

LEDON = 1
LEDOFF = 0

machine.freq(240000000)
print(machine.freq())
# time_start=time.ticks_ms()
# n=0
# while(n < 10000):
#   n=n+1
# time_end=time.ticks_ms()
# print('time cost',time_end-time_start,'ms')
# sta_if = network.WLAN(network.STA_IF)
# if not sta_if.isconnected():
#   print('connecting to network...')
#   sta_if.active(True)
#   sta_if.connect('dlhome-2.4G', '18602824101000')
#   while not sta_if.isconnected():
#     pass
# print('network config:', sta_if.ifconfig())
# 
# sendmsg = 'sadffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffsdffffffffffffffffffffffffffffffffffffffffffsdfffffffffffffffffffffffffffff'

#wdt = WDT(timeout=200) 

#wdt.feed()

LED0 = machine.Pin(2,machine.Pin.OUT)
LED0.value(1)

HOST = ''
PORT = 11111
BUFSIZ = 1024
ADDR = (HOST,PORT)

#tcpSerSock = socket.socket(socket.AF_INET,socket.SOCK_STREAM
tcpSerSock = socket.socket()
tcpSerSock.bind(ADDR)
tcpSerSock.listen(5)
tcpSerSock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)


vspi = SPI(2, baudrate=8000000, polarity=0, phase=0, bits=8, firstbit=0, sck=Pin(18), mosi=Pin(23), miso=Pin(19))
print(vspi)
#spi.init(baudrate=200000) # 设置波特率
p21 = Pin(21, Pin.OUT)
p21.value(1)
utime.sleep_us(1)
p21.value(0)
utime.sleep_us(1)
p21.value(1)
utime.sleep(1)

mode0 = Pin(16, Pin.OUT)
mode1 = Pin(17, Pin.OUT)
mode0.value(1)
mode1.value(1)

pwdn1 = Pin(36, Pin.IN) 
pwdn2 = Pin(39, Pin.IN) 
pwdn3 = Pin(34, Pin.IN) 
pwdn4 = Pin(35, Pin.IN)

print(pwdn1.value())
print(pwdn2.value())
print(pwdn3.value())
print(pwdn4.value())

DRDY = Pin(22, Pin.IN) 

format0 = Pin(25, Pin.IN) 
format1 = Pin(26, Pin.IN) 
format2 = Pin(27, Pin.IN) 
utime.sleep(1)
print(format0.value()) 
print(format1.value()) 
print(format2.value()) 

spi_recv_data = [0, 0, 0, 0]
sample_voltage = [0, 0, 0, 0]



while True:
    #if DRDY.value() == 0 :
        #print(vspi.read(12))            # 在MOSI上读取10字节
        data = vspi.read(6)
        print(data)
        for i in range(0, 2):
            spi_recv_data[i] = (data[3 * i] << 16) + (data[3 * i + 1] << 8) + data[3 * i + 2]

            if spi_recv_data[i] & 0x00800000 :     
                sample_voltage[i] = spi_recv_data[i] - 0x01000000
            else :
                sample_voltage[i] = spi_recv_data[i]
                
            print('{:.6f}'.format(sample_voltage[i]*2.5/8388607))




while True:
    print('waiting for connection...')
    tcpCliSock, addr = tcpSerSock.accept()
    print('...connnecting from:', addr)

    while True:
        data = tcpCliSock.recv(BUFSIZ)
        print (data)
        if not data:
            break
  
        print (LEDON)
        print (LEDOFF)
        if(LEDON==int(data)):
            LED0.value(0)
            while True:
                utime.sleep_us(1)
                #print (sendmsg)
                tcpCliSock.send(sendmsg )
        if(LEDOFF==int(data)):
            LED0.value(1)
        #tcpCliSock.send('[%s] %s' %(bytes(ctime(),'utf-8'),data))
        #tcpCliSock.send(('[%s] %s' % (ctime(), data)).encode())
    tcpCliSock.close()
tcpSerSock.close()

# html='''<!DOCTYPE html>
# <html>
# <head><meta charset='UTF-8'><title>网络led开关服务器</title></head>
# <center><h2>网络LED开关服务器</h2></center>
# <center><form>
# <button name="LED" value='ON' type='submit'> LED ON </button>
# <button name="LED" value='OFF' type='submit'> LED OFF </button>
# </form>
# </center>
# '''
# LED0 = machine.Pin(2,machine.Pin.OUT)
#led off
# LED0.value(1)
# s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
# s.bind(('',80))
# s.listen(5)
# while True:
    # conn,addr=s.accept()
    # print("GOT a connection from %s" % str(addr))
    # request=conn.recv(1024)
    # print("Content %s" % str(request))
    # request=str(request)
    # LEDON=request.find('/?LED=ON')
    # LEDOFF=request.find('/?LED=OFF')
    # if(LEDON==6):
        # LED0.value(0)
    # if(LEDOFF==6):
        # LED0.value(1)
    # response=html
    # conn.send(response)
    # conn.close()    
       

        






