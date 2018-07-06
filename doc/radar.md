# Radar
## Table of contents

- [Introduction](#introduction)
- [Measuring Principle](#measuring-principle)
- [Raw Targets](#raw-targets)
- [Tracking Objects](#tracking-objects)
- [Bugs and feature requests](#bugs-and-feature-requests)
- [Testing](#testing)

## Introduction

This driver supports the radar type RMS3xx. This radar records raw targets and tracking objects. The tracking objects are determined on the basis of the raw targets. Two variants of a tracking method are already installed in the radar, which enables the radar to be put into operation quickly.

## Measuring Principle

The RMS3xx is based on FMCW radar. 
With frequency-modulated continuous wave radar (FMCW radar), the transmission frequency is changed periodically. 
Triangle functions are usually used for distance measurement. 
While the transmission frequency changes as linearly as possible to the target object and back during the propagation time of the signal, 
the signal reflected by the object and received by the radar is time-shifted to the original transmitted frequency. 
By mixing the transmitted signal with the received signal, the frequency shift and thus the time shift can be determined. 
Based on the known modulation parameters of the transmitter, the propagation time of the signal can be determined, which in turn is proportional to the distance of the object. 
For precise distance measurement, therefore, the transmission frequency must be modulated as precisely as possible in linear fashion, 
since any non-linearity impairs the distance accuracy of the radar.

Through this indirect time measurement via the frequency change of the transmitter, even very close targets can be measured with high accuracy and cost-efficiency using the FMCW method, provided that the modulation parameters are selected appropriately. The distance resolution is determined by the bandwidth of the transmitted signal.

## Raw Targets

Raw targets correspond to individual reflectors that are detected by the radar. Each individual reflector carries the following information:
* range
Horizontal angle (azimuth)
Doppler speed
reflectivity of the target

The radar RMS3xx does not resolve elevation angles.  Therefore, the radar assumes the elevation values (z values) with 0.0. The error in distance estimation is usually negligible and is 0.4% (1.0 - cos(5°)) at an elevation angle of 5° compared to horizontal.

## Tracking Objects

Tracking objects are determined from the raw targets via a tracking procedure over the spatial and temporal distribution of the raw targets. The track method estimates the location, direction and speed of the object based on an initial estimate.  After initialization, new raw targets are assigned to the track if they "fit" to the track.  This process is called "gating". Once these raw targets have been assigned to the track, the track is updated and the new estimate is used for further processing.

