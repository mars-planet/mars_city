from UserSkeletonData import UserSkeletonData


def user_skeleton_data(user):
    print '================================================================='
    print 'Head            --> {}  {}  {}'.format(user.head.x, user.head.y,
                                                user.head.z)
    print 'Neck            --> {}  {}  {}'.format(user.neck.x, user.neck.y,
                                                user.neck.z)
    print 'Left shoulder   --> {}  {}  {}'.format(user.left_shoulder.x,
                                user.left_shoulder.y, user.left_shoulder.z)
    print 'Right shoulder  --> {}  {}  {}'.format(user.right_shoulder.x,
                              user.right_shoulder.y, user.right_shoulder.z)
    print 'Left Elbow      --> {}  {}  {}'.format(user.left_elbow.x,
                                      user.left_elbow.y, user.left_elbow.z)
    print 'Right Elbow     --> {}  {}  {}'.format(user.right_elbow.x,
                                    user.right_elbow.y, user.right_elbow.z)
    print 'Left Hand       --> {}  {}  {}'.format(user.left_hand.x,
                                        user.left_hand.y, user.left_hand.z)
    print 'Right Hand      --> {}  {}  {}'.format(user.right_hand.x,
                                      user.right_hand.y, user.right_hand.z)
    print 'Torso           --> {}  {}  {}'.format(user.torso.x, user.torso.y,
                                                              user.torso.z)
    print 'Left Hip        --> {}  {}  {}'.format(user.left_hip.x,
                                          user.left_hip.y, user.left_hip.z)
    print 'Right Hip       --> {}  {}  {}'.format(user.right_hip.x,
                                        user.right_hip.y, user.right_hip.z)
    print 'Left Knee       --> {}  {}  {}'.format(user.left_knee.x,
                                        user.left_knee.y, user.left_knee.z)
    print 'Right Knee      --> {}  {}  {}'.format(user.right_knee.x,
                                      user.right_knee.y, user.right_knee.z)
    print 'Left Foot       --> {}  {}  {}'.format(user.left_foot.x,
                                        user.left_foot.y, user.left_foot.z)
    print 'Right Foot      --> {}  {}  {}'.format(user.right_foot.x,
                                      user.right_foot.y, user.right_foot.z)
    print '================================================================='


user0 = UserSkeletonData()
st = user0.st

while True:
    st.loop()
    if st.usersCount() > 0:
        user_skeleton_data(user0)
