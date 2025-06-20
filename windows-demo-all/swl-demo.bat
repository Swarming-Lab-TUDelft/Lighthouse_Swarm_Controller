@echo off
start wt.exe -w 0 nt cmd /k "usbipd list && usbipd attach --wsl --busid 3-2 && usbipd attach --wsl --busid 3-3 && usbipd attach --wsl --busid 3-4" 
start wt.exe nt ubuntu2004


