/*****************************************************************************
 * SkeletonTracker : Tracks skeleton of user
 * parameter       : maxUser - It is the maximum amount of user to track
 *****************************************************************************/

#include "NiTE.h"
#include <cstdlib>
#include <string>

class SkeletonTracker {
    private:
        int maxUsers;
        nite::UserTracker userTracker;
        nite::Status niteRc;
        nite::UserTrackerFrameRef userTrackerFrame;
        const nite::Array<nite::UserData> *users;

    public:
        SkeletonTracker(int maxUsers) {
            nite::NiTE::initialize();

            this->maxUsers = maxUsers;
            bool color_output = false;
            niteRc = userTracker.create();

            if (isatty(STDOUT_FILENO) && strcmp(getenv("TERM"), "dumb")) {
                color_output = true;
            }

            if (niteRc != nite::STATUS_OK) {
                if (!color_output) {
                    printf("%s", "Couldn't create user tracker\n");
                exit(1);
                }
                else {
                    printf("%s", "\033[1;31mCouldn't create user tracker\n\033[m");
                }
            }

            if (!color_output) {
                printf("\nStart moving around to get detected...\n");
                printf("PSI pose may be required for calibration\n");
            }
            else {
                printf("\n\033[1;34mStart moving around to get detected...\n\033[m");
                printf("\033[1;34mPSI pose may be required for calibration\n\033[m");
            }

            niteRc = userTracker.readFrame(&userTrackerFrame);

            if (niteRc != nite::STATUS_OK) {
                printf("Get next frame failed\n");
            }

            users = &userTrackerFrame.getUsers();
        }

        void loop() {
            niteRc = userTracker.readFrame(&userTrackerFrame);

            if (niteRc != nite::STATUS_OK) {
                printf("Get next frame failed\n");
            }

            users = &userTrackerFrame.getUsers();

            for (int i = 0; i < users->getSize(); ++i) {
                const nite::UserData &user = (*users)[i];
 
                if (user.isNew()) {
                    userTracker.startSkeletonTracking(user.getId());
                }
            }
        }

        int getUsersCount() {
            return users->getSize();
        }

        bool isUserTracked(int i) {
            return ((*users)[i].getSkeleton().getState()
                     == nite::SKELETON_TRACKED);
        }

        float getUserSkeletonHeadConf(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_HEAD).getPositionConfidence();
        }

        float getUserSkeletonHeadX(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_HEAD).getPosition().x;
        }

        float getUserSkeletonHeadY(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_HEAD).getPosition().y;
        }

        float getUserSkeletonHeadZ(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_HEAD).getPosition().z;
        }

        float getUserSkeletonNeckX(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_NECK).getPosition().x;
        }

        float getUserSkeletonNeckY(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_NECK).getPosition().y;
        }

        float getUserSkeletonNeckZ(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_NECK).getPosition().z;
        }

        float getUserSkeletonL_ShX(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER).getPosition().x;
        }

        float getUserSkeletonL_ShY(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER).getPosition().y;
        }

        float getUserSkeletonL_ShZ(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER).getPosition().z;
        }

        float getUserSkeletonR_ShX(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER).getPosition().x;
        }

        float getUserSkeletonR_ShY(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER).getPosition().y;
        }

        float getUserSkeletonR_ShZ(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER).getPosition().z;
        }

        float getUserSkeletonL_ElbowX(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW).getPosition().x;
        }

        float getUserSkeletonL_ElbowY(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW).getPosition().y;
        }

        float getUserSkeletonL_ElbowZ(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW).getPosition().z;
        }

        float getUserSkeletonR_ElbowX(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW).getPosition().x;
        }

        float getUserSkeletonR_ElbowY(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW).getPosition().y;
        }

        float getUserSkeletonR_ElbowZ(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW).getPosition().z;
        }

        float getUserSkeletonL_HandX(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().x;
        }

        float getUserSkeletonL_HandY(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().y;
        }

        float getUserSkeletonL_HandZ(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().z;
        }

        float getUserSkeletonR_HandX(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition().x;
        }

        float getUserSkeletonR_HandY(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition().y;
        }

        float getUserSkeletonR_HandZ(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition().z;
        }

        float getUserSkeletonTorsoX(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_TORSO).getPosition().x;
        }

        float getUserSkeletonTorsoY(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_TORSO).getPosition().y;
        }

        float getUserSkeletonTorsoZ(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_TORSO).getPosition().z;
        }

        float getUserSkeletonL_HipX(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_LEFT_HIP).getPosition().x;
        }

        float getUserSkeletonL_HipY(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_LEFT_HIP).getPosition().y;
        }

        float getUserSkeletonL_HipZ(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_LEFT_HIP).getPosition().z;
        }

        float getUserSkeletonR_HipX(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_RIGHT_HIP).getPosition().x;
        }

        float getUserSkeletonR_HipY(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_RIGHT_HIP).getPosition().y;
        }

        float getUserSkeletonR_HipZ(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_RIGHT_HIP).getPosition().z;
        }

        float getUserSkeletonL_KneeX(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_LEFT_KNEE).getPosition().x;
        }

        float getUserSkeletonL_KneeY(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_LEFT_KNEE).getPosition().y;
        }

        float getUserSkeletonL_KneeZ(int i) {
            return (*users)[i].getSkeleton().getJoint(nite::JOINT_LEFT_KNEE).getPosition().z;
        }

        float getUserSkeletonR_KneeX(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE).getPosition().x;
        }

        float getUserSkeletonR_KneeY(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE).getPosition().y;
        }

        float getUserSkeletonR_KneeZ(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE).getPosition().z;
        }

        float getUserSkeletonL_FootX(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_LEFT_FOOT).getPosition().x;
        }

        float getUserSkeletonL_FootY(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_LEFT_FOOT).getPosition().y;
        }

        float getUserSkeletonL_FootZ(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_LEFT_FOOT).getPosition().z;
        }

        float getUserSkeletonR_FootX(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT).getPosition().x;
        }

        float getUserSkeletonR_FootY(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT).getPosition().y;
        }

        float getUserSkeletonR_FootZ(int i) {
            return
            (*users)[i].getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT).getPosition().z;
        }

        void shutdown(void) {
            nite::NiTE::shutdown();
        }
};
