# Short and long SLERP interpolation
Computes the slerp interpolated path between two quaternions p0 and p1. Because any rotation can be represented by a quaternion p or its opposite -p, the rotation path between two quaternions may be the 'long way' or the 'short way'. This function allows to compute one or the other. If not specified, the short way is computed. Slerp interpolation guarantees a constant angular velocity rotation from the start quaternion to the end. If quaternions are too close to each other (almost parallel), linear interpolation is used instead.

A demo is included that compares a simple rotation path computed with slerp, going the short way or the long way

For more information about short vs long slerp: https://en.wikipedia.org/wiki/Slerp (Quaternion Slerp section)

### Inputs
* p0 - 1x4 double representing quaternion starting point
* p1 - 1x4 double representing quaternion end point
* t - 1xN double representing parameter t, interpolation coefficient from 0 to 1 (inclusive). At t=0, p_interp = p0_unit; at t=1, p_interp=p1_unit;
* path_str - [OPTIONAL, default = 'short'] a string, either 'short' or 'long', indicating whether we want to compute the slerp path the 'short way' or the 'long way'. If not specified, the short way is computed.
* DOT_THRESHOLD - [OPTIONAL, default = 0.9995]. If the absolute value of the dot product between p0 and p1 is above this threshold, the quaternions are considered almost parallel and a linear interpolation is computed instead


### Outputs
* p_interp - Nx4 double, each row representing a normalised quaternion in the interpolated path. Start and end points are included: p_interp(1,:) = p0_unit; p_interp(end,:) = p1_unit;


![alt text](https://github.com/sfmig/short-and-long-slerp/blob/main/fex_cover_image.png?raw=true)
