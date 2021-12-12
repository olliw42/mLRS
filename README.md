# mLRS

This is the mLRS project.

Its goal is an open source 2.4 GHz LORA-based high-performance long-range radio link, which provides fully transparent bidirectional serial connection combined with full remote control.

The 'm' in the project name alludes to 'Mavlink', as it will have features which optimizes performance for Mavlink systems, but it always will also provide a transparent serial link and hence be of wider use. The 'LR' in the project name alludes to 'long range', which however should not be understood in terms of an absolute range, like 50 km or so, but - of course - as the best possible range under the given conditions. Physical laws simply say that the higher the data rate the shorter the range. So, mLRS cannot compete range-wise with systems which achieve their range by reducing data rate to the minimal. The goal of mLRS is to achieve a high range under the conditions of a relatively high data rate. 

Typical specs could be to transmits 16 RC channels at "full" resolution (11 bit for channels 1-4, 8 bit for channels 5-16), and serial data rates of about 3-5 kB/s.

Currently, mLRS is work in progress, and has not yet achieved its goals.

Many LRS or radio links with telemetry exist, among them open source projects such as SiK radios, OpenLRS, ExpressLRS, but also befinitiv wifibroadcast based projects like OpenHD or Ruby, closed source hobbists projects such as UltimateLRS, QczekLRS, as well as commercial systems such as DragonLink, RFD900, Dronee Zoon, Siyi, but also TBS Crossfire and alike.

However, while some of these systems are very close to the goals, none of them satisfy all desired criteria like relatively cheap, 2.4 GHz, biirectional with sufficient data rate, and open source. Hence this project.    
