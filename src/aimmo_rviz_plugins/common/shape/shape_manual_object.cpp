#include "shape_manual_object.h"

ShapeManualObject::ShapeManualObject(Ogre::SceneManager* manager, Ogre::SceneNode* parent_node)
  : Shape(manager, parent_node)
{
  manual_object_ =  sceneManager()->createManualObject();
  sceneNode()->attachObject(manual_object_);
}

ShapeManualObject::~ShapeManualObject()
{
  sceneManager()->destroyManualObject(manual_object_);
}

Ogre::ManualObject* ShapeManualObject::manualObject()
{
  return manual_object_;
}
