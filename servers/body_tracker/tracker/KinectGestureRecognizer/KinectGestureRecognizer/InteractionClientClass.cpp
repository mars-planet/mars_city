#include "stdafx.h"
#include "InteractionClientClass.h"

STDMETHODIMP_(ULONG) InteractionClientClass::AddRef(){ return 1; }

STDMETHODIMP_(ULONG) InteractionClientClass::Release(){ return 0; }

STDMETHODIMP InteractionClientClass::QueryInterface(REFIID riid, void **ppv){
	return S_OK;
}

STDMETHODIMP InteractionClientClass::GetInteractionInfoAtLocation(DWORD skeletonTrackingId,
	NUI_HAND_TYPE handType, FLOAT x, FLOAT y, _Out_ NUI_INTERACTION_INFO *pInteractionInfo)
{

	pInteractionInfo->IsGripTarget = FALSE;
	pInteractionInfo->IsPressTarget = FALSE;

	return S_OK;
}
