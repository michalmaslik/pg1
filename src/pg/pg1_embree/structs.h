#pragma once

struct Vertex3f { float x, y, z; }; // a single vertex position structure matching certain format

using Normal3f = Vertex3f; // a single vertex normal structure matching certain format

struct Coord2f { float u, v; }; // texture coord structure

struct Triangle3ui { unsigned int v0, v1, v2; }; // indicies of a single triangle, the struct must match certain format, e.g. RTC_FORMAT_UINT3

struct RTC_ALIGN(16) Color4f
{
	struct { float r, g, b, a; }; // a = 1 means that the pixel is opaque

	float Compress(const float u, const float gamma = 2.4f) {
		if (u <= 0) {
			return 0.0f;
		}
		if (u >= 1.0f) {
			return 1.0f;
		}
		if (u <= 0.0031308f) {
			return 12.92f * u;
		}

		return (1.055f * powf(u, 1.0f / gamma)) - 0.055f;
	}

	void Compress(const float gamma = 2.4f) {
		r = Compress(r, gamma);
		g = Compress(g, gamma);
		b = Compress(b, gamma);
	}

	float Expand(const float u, const float gamma = 2.4f) {
		if (u <= 0.0f) {
			return 0.0f;
		}
		if (u >= 1.0f) {
			return 1.0f;
		}
		if (u <= 0.04045f) {
			return u / 12.92f;
		}
		return powf((u + 0.055f) / 1.055f, gamma);
	}

	void Expand(const float gamma = 2.4f) {
		r = Expand(r, gamma);
		g = Expand(g, gamma);
		b = Expand(b, gamma);
	}


};

struct Color3f {
	struct { float r, g, b; };

	float Compress(const float u, const float gamma = 2.4f) {
		if (u <= 0.0f) {
			return 0.0f;
		}
		if (u >= 1.0f) {
			return 1.0f;
		}
		if (u <= 0.0031308f) {
			return 12.92f * u;
		}

		return (1.055f * powf(u, 1.0f / gamma)) - 0.055f;
	}

	void Compress(const float gamma = 2.4f) {
		r = Compress(r, gamma);
		g = Compress(g, gamma);
		b = Compress(b, gamma);
	}

	float Expand(const float u, const float gamma = 2.4f) {
		if (u <= 0.0f) {
			return 0.0f;
		}
		if (u >= 1.0f) {
			return 1.0f;
		}
		if (u <= 0.04045f) {
			return u / 12.92f;
		}
		return powf((u + 0.055f) / 1.055f, gamma);
	}

	void Expand(const float gamma = 2.4f) {
		r = Expand(r, gamma);
		g = Expand(g, gamma);
		b = Expand(b, gamma);
	}

	/*
	explicit operator Vector3() const {
		Color3f c = Color3f{ r, g, b };
		c.expand();
		return Vector3(c);
	};*/

};

