# theyseemerolling-software
please do not put sudo before openocd, please run  
sudo usermod -a -G uucp <user> for serial monitor  
for udev rule:  
nano /etc/udev/rules.d/70-st-link.rules  
```
  #NUCLEO ST-LINK/V2.1  
  ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b",TAG+="uaccess"  
```
to find theses values: lsusb  
then udevadm control --reload-rules  
unplug and plug st-link  

#for the alternate functions:  
https://www.st.com/resource/en/datasheet/stm32f303zd.pdf  
page 53 to 63
