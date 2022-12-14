====================================
README FOR LZROVERENGINE LUNAR EMULATION SUBSYSTEM
====================================

This subsystem is meant to be responsible for emulating everything that in a real mission scenario would be captured by the rover.
This includes it's sensors, which currently consists of optical sensors returning an image containing the colors in front and TOF/Lidar sensors returning a set of 3D points on the surface.
This also includes an estimation of the rovers relative position and motion given a set of locomotion instructions. 

All usage of the subsystem should be done through the LZEngine class. Rover information is exclusively retrieved via function calls returning their appropriate data structs.

For debugging reasons, this subsystem also allows for a window to be created with a user controlled camera on top of a set of more basic debugging functions, for runtime inspection of the subsystem.

====================================
FOOTMARKS
====================================

IDK MAN WHACKADOO YOUR PLOBLEM NOW >:)