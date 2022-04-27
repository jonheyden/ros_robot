# Arduino Code Differences
The robot2.ino and robotedit2.ino are slightly different.
robotedit2.ino uses the [sabertooth](https://www.dimensionengineering.com/info/arduino) library which simplifies commands and seems to bring a sense of stability to the platform.

This method diverges from the PID loop and instead uses linear regression to give velocity commands. 
Three points were taken that related to the assumed tick speed of ```24 ticks = 1 m/s```

This gave roughly the following:
```24 ticks = 120 Commanded Speed```
```12 ticks = 70 Commanded Speed```
```1 tick = 33 Commanded Speed```

Putting these values into a linear regression algorithm gave the values used in the robotedit2.ino starting on line 161. 

The values returned from the regression were ```3.625*x + 30.83``` for the formula. 