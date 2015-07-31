#pragma once

#include <Windows.h>
#include <NuiApi.h>
#include <KinectInteraction.h>

class InteractionClientClass : public INuiInteractionClient
{
public:
	STDMETHODIMP_(ULONG) AddRef();
	STDMETHODIMP_(ULONG) Release();
	STDMETHODIMP QueryInterface(REFIID riid, void **ppv);

	STDMETHODIMP GetInteractionInfoAtLocation(DWORD skeletonTrackingId,
		NUI_HAND_TYPE handType, FLOAT x, FLOAT y, _Out_ NUI_INTERACTION_INFO *pInteractionInfo);
};