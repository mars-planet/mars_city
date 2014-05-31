Error due to bad link

Old Method:
```
gazebo::Body* body = dynamic_cast<gazebo::body*>(gazebo::World::Instance()->GetEntityByName(req.link_name));
if (body == NULL)
```
New Method:
```
gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::link>(this->world->GetEntity(req.link_name));
if (body)
```
This results in an error on deleting the model from Gazebo world. The error is
```
gzserver: /usr/include/boost/smart_ptr/shared_ptr.hpp:418: T* boost::shared_ptr<T>::operator->() const [with T = gazebo::physics::World]: Assertion `px != 0' failed.
```