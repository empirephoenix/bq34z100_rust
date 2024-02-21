# bq34z100
Provides an embedded_hal driver for the fuel gauge series bq34z100

Out of the box it is no_std and only allows reading the various registers, enough for usual client applications.
It has a "write" feature, that currently requires std for some math functions. With write it is possible to update various parameters of the chip.
Setting up the chip is limited, the default chemid uses is ok compatible with most lipo and liion cells. It is possible to set the capacity and design energy but without an EV2400/EV2300 programmer it is not possible to change for eg. LiFePo. For most batteries it might be easier to just buy a bootleg programmer and set up all static value and start the calibrations via it. Then only use this to read the registers from a MCU to get state informations.

Please note that this is mostly a direct port of https://github.com/xkam1x/BQ34Z100G1/blob/master/bq34z100g1.cpp and using some parts of the xemics conversion from https://github.com/Ralim/BQ34Z100/blob/master/bq34z100.cpp. As a result this is not ideomatic rust code.

Please note, that the library is still very rough at the edges and pull requests are welcome to improve it :)