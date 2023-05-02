#ifndef SHAPE_MANUAL_OBJECT_H
#define SHAPE_MANUAL_OBJECT_H

#include <OgreManualObject.h>
#include "shape.h"

class ShapeManualObject : public Shape
{
public:
  ShapeManualObject(Ogre::SceneManager* manager, Ogre::SceneNode* parent_node = NULL);
  virtual ~ShapeManualObject();

protected:
  Ogre::ManualObject* manualObject();

private:
  Ogre::ManualObject* manual_object_;
};

typedef std::shared_ptr<ShapeManualObject> ShapeManualObjectPtr;

#endif // SHAPE_MANUAL_OBJECT_H
