#pragma once

#include <Windows.h>
#include <NuiApi.h>
#include <KinectInteraction.h>

#define HAND_STATUS_OPEN 0
#define HAND_STATUS_CLOSED 1

class KinectGestureRecognizer
{
public:
	KinectGestureRecognizer(INuiSensor* sensor, HANDLE depthStreamHandle, HANDLE depthEvent, HANDLE skeletonEvent);

	void ProcessDepth(NUI_IMAGE_FRAME depthFrame);
	void ProcessSkeleton(NUI_SKELETON_FRAME skeletonFrame);

	void startRecognition();

	int getRightHandStatus();
	int getLeftHandStatus();
private:
	INuiSensor* pNuiSensor;
	INuiInteractionStream* interactionStream;
	HANDLE depthStreamHandle;

	int rightHandStatus;
	int leftHandStatus;

	HANDLE depthEvent;
	HANDLE skeletonEvent;

	BYTE* pDepthBuffer;

	void ProcessInteraction();
};

#define DLLEXPORT __declspec(dllexport)

extern "C" {
	DLLEXPORT KinectGestureRecognizer* __stdcall new_KinectGestureRecognizer(INuiSensor** sensor, HANDLE depthStreamHandle, HANDLE depthEvent, HANDLE skeletonEvent);

	DLLEXPORT void __stdcall c_ProcessDepth(KinectGestureRecognizer* kgr, NUI_IMAGE_FRAME* depthFrame);
	DLLEXPORT void __stdcall c_ProcessSkeleton(KinectGestureRecognizer* kgr, NUI_SKELETON_FRAME* skeletonFrame);

	DLLEXPORT void __stdcall c_startRecognition(KinectGestureRecognizer* kgr);

	DLLEXPORT int __stdcall c_getRightHandStatus(KinectGestureRecognizer* kgr);
	DLLEXPORT int __stdcall c_getLeftHandStatus(KinectGestureRecognizer* kgr);
}