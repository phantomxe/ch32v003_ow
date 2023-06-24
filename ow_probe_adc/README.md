## WIP

I'm trying use ch32v003 as onewire slave device. I got some problems about timing. Sometimes reset not detected or write mode not disabled. 

Here is a screenshot of logic analyzer when debugging the code.
![ow demo 1](https://github.com/phantomxe/ch32v003_ow/assets/22988043/9dd07235-3422-4bb5-beec-d39bb7555493)



But it can fail for some reason. It didn't respond the master on reset state.

![ow fail sometimes](https://github.com/phantomxe/ch32v003_ow/assets/22988043/0d3e5bda-8ade-47b2-8938-f69e282be0eb)
