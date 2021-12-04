# vex-tipping-up
GitHub repository for VEX Robotics team 3018E Paracord for the 2021-2022 season: Tipping Point.

## Features
* Macro class
  * Macros run in separate threads and can have blocking code
  * Safely terminate using an exception caught by a custom wait function
  * Sub macros are simultaneously terminated
  * Clean up function that runs even if terminated
  * Macro groups prevent conflicting macros from running simultaneously

* Button handler
  * Assign functions and macros to pressed and released events
  * Buttons can be reassigned
  * Button groups enable clearing catagories of button assignments

* Controller Menu
  * Autonomous selection
  * Autonomous testing
  * Supports recursive folders
  * Side scrolling if more items are in a folder than can fit on screen
