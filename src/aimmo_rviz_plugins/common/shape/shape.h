#ifndef SHAPE_H
#define SHAPE_H

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreMaterial.h>

#include <memory>

class Shape
{
public:
  Shape(Ogre::SceneManager* manager, Ogre::SceneNode* parent_node = NULL);
  virtual ~Shape();

public:
  virtual void update() = 0;

protected:
  Ogre::SceneManager* sceneManager();
  Ogre::SceneNode* sceneNode();
  Ogre::MaterialPtr material();

private:
  Ogre::SceneManager* scene_manager_;
  Ogre::SceneNode* scene_node_;  
  Ogre::MaterialPtr material_;
};

typedef std::shared_ptr<Shape> ShapePtr;

#endif // SHAPE_H
