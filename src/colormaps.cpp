#include "colormaps.h"

RgbColor GetJetColour(float v, float vmin, float vmax, float maxColor)
{
	if (v < vmin)
		v = vmin;
	if (v > vmax)
		v = vmax;

	double dv = vmax - vmin;

	if (v < (vmin + 0.25 * dv))
		return RgbColor(
			0, 
			(int)(maxColor * (4 * (v - vmin) / dv)),
			(int)maxColor);

	if (v < (vmin + 0.5 * dv))
		return RgbColor(
			0,
			(int)maxColor,
			(int)(maxColor * (1 + 4 * (vmin + 0.25 * dv - v) / dv)));

	if (v < (vmin + 0.75 * dv))
		return RgbColor(
			(int)(maxColor * (4 * (v - vmin - 0.5 * dv) / dv)),
			(int)maxColor,
			0);

	return RgbColor(
		(int)maxColor,
		(int)(maxColor * (1 + 4 * (vmin + 0.75 * dv - v) / dv)),
		0);
}
