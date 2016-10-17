#!/bin/sh

for i in `seq 1 19`; do 
       fn=`printf file_%02d $i`; 
       read sent; echo $sent; 
       rec -r 16000 -e signed-integer -b 16 -c 1 $fn.wav 2>/dev/null; 
done < a.txt

