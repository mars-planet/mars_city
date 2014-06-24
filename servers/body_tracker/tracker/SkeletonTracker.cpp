#include "SkeletonTracker.h"

extern "C" {
    SkeletonTracker *SkeletonTracker_new() {
        return new SkeletonTracker(4);
    }

    void loop(SkeletonTracker *SkeletonTracker) {
        SkeletonTracker->loop();
    }

    int getUsersCount(SkeletonTracker *SkeletonTracker) {
        return SkeletonTracker->getUsersCount();
    }

    bool isUserTracked(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->isUserTracked(i);
    }

    float getUserSkeletonHeadConf(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonHeadConf(i);
    }

    float getUserSkeletonHeadX(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonHeadX(i);
    }

    float getUserSkeletonHeadY(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonHeadY(i);
    }

    float getUserSkeletonHeadZ(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonHeadZ(i);
    }

    float getUserSkeletonNeckX(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonNeckX(i);
    }

    float getUserSkeletonNeckY(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonNeckY(i);
    }

    float getUserSkeletonNeckZ(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonNeckZ(i);
    }


    float getUserSkeletonL_ShX(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonL_ShX(i);
    }

    float getUserSkeletonL_ShY(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonL_ShY(i);
    }

    float getUserSkeletonL_ShZ(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonL_ShZ(i);
    }

    float getUserSkeletonR_ShX(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonR_ShX(i);
    }

    float getUserSkeletonR_ShY(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonR_ShY(i);
    }

    float getUserSkeletonR_ShZ(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonR_ShZ(i);
    }

    float getUserSkeletonL_ElbowX(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonL_ElbowX(i);
    }

    float getUserSkeletonL_ElbowY(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonL_ElbowY(i);
    }

    float getUserSkeletonL_ElbowZ(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonL_ElbowZ(i);
    }

    float getUserSkeletonR_ElbowX(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonR_ElbowX(i);
    }

    float getUserSkeletonR_ElbowY(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonR_ElbowY(i);
    }

    float getUserSkeletonR_ElbowZ(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonR_ElbowZ(i);
    }

    float getUserSkeletonL_HandX(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonL_HandX(i);
    }

    float getUserSkeletonL_HandY(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonL_HandY(i);
    }

    float getUserSkeletonL_HandZ(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonL_HandZ(i);
    }

    float getUserSkeletonR_HandX(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonR_HandX(i);
    }

    float getUserSkeletonR_HandY(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonR_HandY(i);
    }

    float getUserSkeletonR_HandZ(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonR_HandZ(i);
    }

    float getUserSkeletonTorsoX(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonTorsoX(i);
    }

    float getUserSkeletonTorsoY(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonTorsoY(i);
    }

    float getUserSkeletonTorsoZ(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonTorsoZ(i);
    }

    float getUserSkeletonL_HipX(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonL_HipX(i);
    }

    float getUserSkeletonL_HipY(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonL_HipY(i);
    }

    float getUserSkeletonL_HipZ(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonL_HipZ(i);
    }

    float getUserSkeletonR_HipX(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonR_HipX(i);
    }

    float getUserSkeletonR_HipY(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonR_HipY(i);
    }

    float getUserSkeletonR_HipZ(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonR_HipZ(i);
    }

    float getUserSkeletonL_KneeX(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonL_KneeX(i);
    }

    float getUserSkeletonL_KneeY(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonL_KneeY(i);
    }

    float getUserSkeletonL_KneeZ(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonL_KneeZ(i);
    }

    float getUserSkeletonR_KneeX(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonR_KneeX(i);
    }

    float getUserSkeletonR_KneeY(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonR_KneeY(i);
    }

    float getUserSkeletonR_KneeZ(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonR_KneeZ(i);
    }

    float getUserSkeletonL_FootX(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonL_FootX(i);
    }

    float getUserSkeletonL_FootY(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonL_FootY(i);
    }

    float getUserSkeletonL_FootZ(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonL_FootZ(i);
    }

    float getUserSkeletonR_FootX(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonR_FootX(i);
    }

    float getUserSkeletonR_FootY(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonR_FootY(i);
    }

    float getUserSkeletonR_FootZ(SkeletonTracker *SkeletonTracker, int i) {
        return SkeletonTracker->getUserSkeletonR_FootZ(i);
    }

    void shutdown(SkeletonTracker *SkeletonTracker) {
        SkeletonTracker->shutdown();
    }
}
