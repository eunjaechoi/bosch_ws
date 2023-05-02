#include "lines.h"

Lines::Lines(Ogre::SceneManager *manager, Ogre::SceneNode *parent_node)
    : ShapeManualObject(manager, parent_node)
{
}

void Lines::update()
{
  manualObject()->clear();
  manualObject()->begin(material()->getName(), Ogre::RenderOperation::OT_LINE_STRIP);

  for (auto point : points_)
  {
    manualObject()->position(point);
    manualObject()->colour(color_);
  }

  manualObject()->end();
}
