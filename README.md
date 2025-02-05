This repository contains my implementation of the NYU embedded systems challenge of Fall 2024. In a nutshell, the objective of the challenge was to develop a gesture-based authentication system using an STM32f429iDiscovery board. 

In a more technical sense:
 - I implemented a real-time motion tracking by interfacing with STM32’s SPI bus to read gyroscope data using register configurations from the STM32F4 datasheet. Furthermore I used signal processing techniques, which included using a moving average filter, to smooth and normalize motion data.
 - I also used flash memory to securely store the gesture sequences, which would then be put through a rigorous comparison against a predefined "password". The "password" for this embedded system is a cross (✝).

Attached to this reposity you will find the source code, a debug file I frequently used during testing, as well as a video demo in the form of a YT video. Configuration, as well as package manegement, was all done in the back end thanks to the help of platformIO. 

Demo Link: https://youtube.com/shorts/1AWfBEZszOg

