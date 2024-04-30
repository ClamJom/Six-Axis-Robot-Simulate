#pragma once
#pragma once
#include "cmath"

//颜色定义，用于处理HSV和RGB颜色
namespace MyColor {
	struct CHSV {
		int h; int s; int v;
		CHSV() { this->h = 0; this->s = 0; this->v = 0; }
		CHSV(int h, int s, int v) { this->h = h; this->s = s; this->v = v; }
	};

	struct CRGB {
		int r;
		int g;
		int b;
		CRGB() { this->r = this->g = this->b = 0; }
		CRGB(int r, int g, int b) { this->r = r; this->g = g; this->b = b; }
		void change_to_bgr() {
			int t = this->r;
			this->r = this->b;
			this->b = t;
		}
	};

	static void hsv2rgb(CHSV source, CRGB& target) {
		if (source.s == 0) {
			target.r = source.v / 100 * 255;
			target.g = source.v / 100 * 255;
			target.b = source.v / 100 * 255;
			return;
		}
		float v = (float)source.v / 100;
		float s = (float)source.s / 100;
		float h = (float)source.h;
		float c = v * s;
		float _t = h / 60 - int((h / 60) / 2) * 2;
		float x = c * (1 - abs(_t - 1));
		float m = v - c;
		int i = h / 60;
		float r1 = 0, g1 = 0, b1 = 0;
		switch (i) {
		case 0: {
			r1 = c;
			g1 = x;
			b1 = 0;
			break;
		}case 1: {
			r1 = x;
			g1 = c;
			b1 = 0;
			break;
		}case 2: {
			r1 = 0;
			g1 = c;
			b1 = x;
			break;
		}case 3: {
			r1 = 0;
			g1 = x;
			b1 = c;
			break;
		}case 4: {
			r1 = x;
			g1 = 0;
			b1 = c;
			break;
		}case 5: {
			r1 = c;
			g1 = 0;
			b1 = x;
			break;
		}
		}
		target.r = (r1 + m) * 255;
		target.b = (b1 + m) * 255;
		target.g = (g1 + m) * 255;
	}

	static void rgb2hsv(CRGB source, CHSV& target) {
		float r = source.r;
		float g = source.g;
		float b = source.b;
		float _max;
		float _min;
		r >= g ? r >= b ? _max = r : _max = b : g >= b ? _max = g : _max = b;
		r <= g ? r <= b ? _min = r : _min = b : g <= b ? _min = g : _min = b;
		if (_max == _min)
			target.h = 0;
		else if (_max == r)
			if (g >= b)
				target.h = 60 * (g - b) / (_max - _min);
			else
				target.h = 60 * (g - b) / (_max - _min) + 360;
		else if (_max == g)
			target.h = 60 * (b - r) / (_max - _min) + 120;
		else
			target.h = 60 * (r - g) / (_max - _min) + 240;
		_max == 0 ? target.s = 0 : target.s = (_max - _min) / _max;
		target.v = _max;
	}
}