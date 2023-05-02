#include "shape.h"

#include <OgreSharedPtr.h>
#include <OgreEntity.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>

#include <string>
using namespace std;

Shape::Shape(Ogre::SceneManager *manager, Ogre::SceneNode *parent_node)
{
  scene_manager_ = manager;

  if (!parent_node)
  {
    parent_node = manager->getRootSceneNode();
  }

  scene_node_ = parent_node->createChildSceneNode();

  static int count = 0;
  std::stringstream ss;
  ss << "ShapeMaterial" << count++;

  material_ = Ogre::MaterialManager::getSingleton().create(ss.str(),
                                                           Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(false);
  material_->setCullingMode(Ogre::CULL_NONE);
  material_->setSceneBlending(Ogre::SceneBlendType::SBT_TRANSPARENT_ALPHA);
}

Shape::~Shape()
{
  if (scene_node_->getParentSceneNode())
  {
    scene_node_->getParentSceneNode()->removeChild(scene_node_);
  }
  scene_manager_->destroySceneNode(scene_node_);

  Ogre::MaterialManager::getSingleton().remove(material_->getName());
}

Ogre::SceneManager *Shape::sceneManager()
{
  return scene_manager_;
}

Ogre::SceneNode *Shape::sceneNode()
{
  return scene_node_;
}

Ogre::MaterialPtr Shape::material()
{
  return material_;
}
