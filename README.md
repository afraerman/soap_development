# SOAP: Spacecraft Orbit and Attitude Prediction tool

This software was developed in the Laboratory of Ballistic And Navigation Support For Space Projects of Astro Space Center of PN Lebedev Physics Institute

## Required Files:

- Due to its large size, the ephemeris file must be downloaded into the "Files" folder from the website https://naif.jpl.nasa.gov/pub/naif/generic_kernels/spk/planets/ (de440.bsp is used by default).
- For the same reason, the EGM2008.dat file contains information on harmonics only up to the 500th order.

## Dependencies

- jsoncpp
- Qt

## Build options -- CMAKE
# Windows
- specify paths to jsoncpp and Qt files in CMakeUserPresets.json
- set SOAP_BUILD_GUI ON to choose a GUI version of SOAP or OFF to build a console version

# Linux
- build by default is console (no GUI)
- use -DSOAP_BUILD_GUI=ON flag to enable build with GUI
