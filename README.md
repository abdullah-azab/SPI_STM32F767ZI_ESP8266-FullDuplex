# SPI STM32 F767ZI and ESP8266 Full Duplex

this is an experiment with **STM32 F767ZI Nucleo board** as a master and the **Node MCU ESP 8266 board** as slave featuring full duplex **SPI** communication . 


# 

the required behavior from this simple experiment is to send a string to the slave then verify the string is correct and then retransmit it on request back to the master.

## Frame #1

to start the transmutation master sends the 0x02 (send command) followed by the address which in our case we can leave as 0x00 and then the data is sent making sure to send 32 bytes of data padding with zeros  if need be.
![master to slave first frame](/img/Screenshot%202021-10-15%20065722.png)

##  Frame #2

![ slave to master frame](/img/Screenshot%202021-10-15%20065617.png)
