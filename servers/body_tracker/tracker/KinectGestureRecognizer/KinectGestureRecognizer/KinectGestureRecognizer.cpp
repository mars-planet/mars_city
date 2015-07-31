#include "stdafx.h"
#include "KinectGestureRecognizer.h"
#include "InteractionClientClass.h"

#include <iostream>
using namespace std;

KinectGestureRecognizer::KinectGestureRecognizer(INuiSensor* sensor, HANDLE depthStreamHandle, HANDLE depthEvent, HANDLE skeletonEvent)
{
	//cout << "enter constructor" << endl;

	pNuiSensor = sensor;

	leftHandStatus = HAND_STATUS_OPEN;
	rightHandStatus = HAND_STATUS_OPEN;

	this->depthEvent = depthEvent;
	this->skeletonEvent = skeletonEvent;

	this->depthStreamHandle = depthStreamHandle;
}

void KinectGestureRecognizer::startRecognition() {
	//cout << "enter startRecognition" << endl;

	HANDLE interactionEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	InteractionClientClass interactionClient;
	HRESULT hr = NuiCreateInteractionStream(pNuiSensor, (INuiInteractionClient *)&interactionClient, &interactionStream);
	interactionStream->Enable(interactionEvent);
	if (FAILED(hr)) {
		NuiShutdown();
		return;
	}

	//cout << "interactionStream initialized successfully" << endl;

	HANDLE hEvents[3] = { depthEvent, skeletonEvent, interactionEvent };
	while (1)
	{
		int nEventIdx;
		nEventIdx = WaitForMultipleObjects(sizeof(hEvents) / sizeof(hEvents[0]), hEvents, FALSE, INFINITE);

		if (WAIT_OBJECT_0 == WaitForSingleObject(depthEvent, 0)) {
			// do nothing
		}
		if (WAIT_OBJECT_0 == WaitForSingleObject(skeletonEvent, 0)) {
			// do nothing
		}
		if (WAIT_OBJECT_0 == WaitForSingleObject(interactionEvent, 0))
			ProcessInteraction();
	}
}

void KinectGestureRecognizer::ProcessInteraction()
{
	//cout << "enter ProcessInteraction" << endl;

	HRESULT hr;
	NUI_INTERACTION_FRAME interactionFrame;
	hr = interactionStream->GetNextFrame(0, &interactionFrame);
	/*
	if (FAILED(hr))
	{
		cout << "Process Interaction failed - code: " << hr << endl;
	}
	else
	{
		cout << "Process Interaction OK" << endl;
	}
	*/
	if (FAILED(hr)) return;

	for (int i = 0; i < NUI_SKELETON_COUNT; i++)
	{
		NUI_USER_INFO user = interactionFrame.UserInfos[i];

		if (user.SkeletonTrackingId != 0)
		{
			for (int j = 0; j < NUI_USER_HANDPOINTER_COUNT; j++)
			{
				NUI_HANDPOINTER_INFO hand = user.HandPointerInfos[j];
				NUI_HANDPOINTER_STATE state = (NUI_HANDPOINTER_STATE)hand.State;

				/*
				if (state & NUI_HANDPOINTER_STATE_INTERACTIVE)
				printf("Interactive: ");
				//cout << "Interactive: " << hand.X << " " << hand.Y <<endl;
				if (state & NUI_HANDPOINTER_STATE_PRESSED)
				//cout << "Pressed Button" << endl;
				*/

				if (hand.State != NUI_HANDPOINTER_STATE_NOT_TRACKED)
				{
					//if(m_pInteractionStreamFunction != NULL)
					//	m_pInteractionStreamFunction(hand);
					switch (hand.HandEventType)
					{
					case 0:
						//cout << hand.HandType << " NUI_HAND_EVENT_TYPE_NONE" << endl;
						break;
					case 1:
						cout << hand.HandType << " NUI_HAND_EVENT_TYPE_GRIP" << endl;
						if (hand.HandType == NUI_HAND_TYPE_LEFT) {
							leftHandStatus = HAND_STATUS_CLOSED;
						}
						else if (hand.HandType == NUI_HAND_TYPE_RIGHT) {
							rightHandStatus = HAND_STATUS_CLOSED;
						}
						break;
					case 2:
						cout << hand.HandType << " NUI_HAND_EVENT_TYPE_GRIPRELEASE" << endl;
						if (hand.HandType == NUI_HAND_TYPE_LEFT) {
							leftHandStatus = HAND_STATUS_OPEN;
						}
						else if (hand.HandType == NUI_HAND_TYPE_RIGHT) {
							rightHandStatus = HAND_STATUS_OPEN;
						}
						break;
					}
				}
			}
		}
	}

}

void KinectGestureRecognizer::ProcessDepth(NUI_IMAGE_FRAME depthFrame)
{
	//cout << "enter ProcessDepth" << endl;
	INuiFrameTexture* pDepthImagePixelFrame;
	BOOL nearMode = FALSE;
	pNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(depthStreamHandle, &depthFrame, &nearMode, &pDepthImagePixelFrame);
	NUI_LOCKED_RECT LockedRect;
	pDepthImagePixelFrame->LockRect(0, &LockedRect, NULL, 0);
	if (LockedRect.Pitch != 0)
	{
		HRESULT hr = interactionStream->ProcessDepth(LockedRect.size, PBYTE(LockedRect.pBits), depthFrame.liTimeStamp);
		/*
		if (FAILED(hr))
		{
			cout << "Process Depth failed - code: " << hr << endl;
		}
		else
		{
			cout << "Process Depth OK" << endl;
		}
		*/
	}
	pDepthImagePixelFrame->UnlockRect(0);
}

void KinectGestureRecognizer::ProcessSkeleton(NUI_SKELETON_FRAME skeletonFrame)
{
	//cout << "enter ProcessSkeleton" << endl;

	Vector4 v;
	pNuiSensor->NuiAccelerometerGetCurrentReading(&v);
	HRESULT hr = interactionStream->ProcessSkeleton(NUI_SKELETON_COUNT, skeletonFrame.SkeletonData, &v, skeletonFrame.liTimeStamp);
	/*
	if (FAILED(hr))
	{
		cout << "Process Skeleton failed - code: " << hr << endl;
	}
	else
	{
		cout << "Process Skeleton OK" << endl;
	}
	*/
}

int KinectGestureRecognizer::getRightHandStatus() { return rightHandStatus; }
int KinectGestureRecognizer::getLeftHandStatus() { return leftHandStatus; }

KinectGestureRecognizer* __stdcall new_KinectGestureRecognizer(INuiSensor** sensor, HANDLE depthStreamHandle, HANDLE depthEvent, HANDLE skeletonEvent) {
	//#pragma comment(linker, "/EXPORT:"__FUNCTION__"="__FUNCDNAME__)
	return new KinectGestureRecognizer(*sensor, depthStreamHandle, depthEvent, skeletonEvent);
}

void __stdcall c_ProcessDepth(KinectGestureRecognizer* kgr, NUI_IMAGE_FRAME* depthFrame) {
	//#pragma comment(linker, "/EXPORT:"__FUNCTION__"="__FUNCDNAME__)
	kgr->ProcessDepth(*depthFrame);
}
void __stdcall c_ProcessSkeleton(KinectGestureRecognizer* kgr, NUI_SKELETON_FRAME* skeletonFrame) {
	//#pragma comment(linker, "/EXPORT:"__FUNCTION__"="__FUNCDNAME__)
	kgr->ProcessSkeleton(*skeletonFrame);
}

void __stdcall c_startRecognition(KinectGestureRecognizer* kgr) {
	//#pragma comment(linker, "/EXPORT:"__FUNCTION__"="__FUNCDNAME__)
	kgr->startRecognition();
}

int __stdcall c_getRightHandStatus(KinectGestureRecognizer* kgr) {
	//#pragma comment(linker, "/EXPORT:"__FUNCTION__"="__FUNCDNAME__)
	return kgr->getRightHandStatus();
}
int __stdcall c_getLeftHandStatus(KinectGestureRecognizer* kgr) {
	//#pragma comment(linker, "/EXPORT:"__FUNCTION__"="__FUNCDNAME__)
	return kgr->getLeftHandStatus();
}