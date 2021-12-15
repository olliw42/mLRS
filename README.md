# mLRS

This is the mLRS project.

Its goal is an open source 2.4 GHz LORA-based high-performance long-range radio link, which provides fully transparent bidirectional serial connection combined with full remote control.

The 'm' in the project name alludes to 'Mavlink', as it will have features which optimizes performance for Mavlink systems. However, it always will also provide a transparent serial link and hence will be of wider use and by no means be limited to Mavlink systems only. The 'LR' in the project name alludes to 'long range', which however should not be understood in terms of an absolute range, like 50 km or so, but - of course - as the best possible range under the given conditions. Physical laws simply say that the higher the data rate the shorter the range. So, mLRS cannot compete range-wise with systems which achieve their range by reducing data rate to the minimal, taking according compromises. The goal of mLRS is to achieve a high range under the condition of a relatively high data rate. 

Typical specs could be 14 RC channels at "full" resolution (11 bit for channels 1-4, 8 bit for channels 5-14) with 50 Hz update rate, and serial data rates of about 3-5 kB/s.

Currently, mLRS is work in progress, and there is still a mile to go to achieve its goals.

Many LRS or radio links with telemetry exist, among them open source projects such as SiK radios, OpenLRS, ExpressLRS, but also befinitiv wifibroadcast based projects like OpenHD or Ruby, closed source hobbyist projects such as UltimateLRS, QczekLRS, as well as commercial systems such as DragonLink, RFD900, Dronee Zoon, Siyi, but also TBS Crossfire and alike.

However, while all these systems are truely excellent and achieve their goals, and some of them are indeed very close to what the project aims at, none of them check all points, like relatively cheap, 2.4 GHz, full-duplex with sufficient data rate, open source, and rich features for Mavlink systems. Hence this project.
