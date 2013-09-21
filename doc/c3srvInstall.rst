C3 Central Server Setup
=======================

.. highlightlang:: sh

This page documents how to install the C3 Central Server machine  

SSH
-----

From the provider internal setup page http://192.168.1.1/weblogin.htm
go to open prts and define the addres and port to be open
22 for ssh and 80 fo web server

Then
Set to the C3 server the fixed IP address:
192.168.1.13


- Install ssh daemon
sudo apt-get install openssh-server
sudo restart ssh


chech the machine is accessible from outside at the public IP addres given by the provider:
 77.43.42.209



