#pragma once
#ifndef SETTINGS_H
#define SETTINGS_H

#ifdef WIN
 #ifdef _EXPORT
 #define _API __declspec(dllexport)
 #else
 #define _API __declspec(dllimport)
 #endif
#else
 #define _API 
#endif
#include <memory>

struct _API Settings {

	Settings();
	// Camera calibration and distortion parameters (OpenCV) 
    float fx;
    float fy;
    float cx;
    float cy;

	float k1;
	float k2;
	float k3;
	float p1;
	float p2;

	// SLAM Settings
	float bf; // stereo baseline times fx
	float ThDepth; // Close/Far threshold. Baseline times.
    float DepthMapFactor; // Deptmap values factor

	// Orb Features Detector settings
	float nFeatures;	
	float scaleFactor;
	float nLevels;
	float iniThFAST;
	float minThFAST;
	int nfeatures;
	int edgeThreshold;
	int firstLevel;
	int wtaK;
	bool fastScore;
	int patchSize;
	int fastThreshold;
};

struct _API DeviceSettings {
	enum DeviceType {
		ipad2Front,
		ipad3Rear,
		ipad3Front,
		macbookpro2009
	};
	std::shared_ptr<Settings> GetSettings(DeviceType type);
};

#endif // SETTINGS_H