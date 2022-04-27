# Arduino Code Differences
The robot2.ino and robotedit2.ino are slightly different.
robotedit2.ino uses the [sabertooth](https://www.dimensionengineering.com/info/arduino) library which simplifies commands and seems to bring a sense of stability to the platform.

This method diverges from the PID loop and instead uses linear regression to give velocity commands. Three points were taken that related to the assumed tick speed of 24 ticks = 1 m/s
For this specific motor/encoder/driver, it was about 33 commanded speed before the bot would even start moving. About 70 for 12 ticks, and 120 for 24 ticks. Putting these values into a linear regression algorithm gave the values used in the robotedit2.ino starting on line 161. 