#include "circle.h"

Circle::Circle(Ogre::SceneManager *manager, Ogre::SceneNode *parent_node)
    : ShapeManualObject(manager, parent_node)
{
  step_ = 72;
  fill_ = false;
  point_ = {0, 0, 0};
}

void Circle::update()
{
  manualObject()->clear();

  if (fill_)
  {
    manualObject()->begin(material()->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

    Ogre::Vector3 p0 = point_;
    Ogre::Vector3 p1, p2;
    double accuracy = (2.0 * Ogre::Math::PI) / (double)step_;
    for (int i = 0; i <= step_; i++)
    {
      double theta = 0;
      if (i > 0)
        theta = (double)i * accuracy;

      double x = point_.x + (radius_ * cos(theta));
      double y = point_.y + (radius_ * sin(theta));
      p2 = Ogre::Vector3(x, y, 0);

      if (i == 0)
      {
        p1 = p2;
        continue;
      }

      manualObject()->position(p0);
      manualObject()->colour(color_);

      manualObject()->position(p1);
      manualObject()->colour(color_);

      manualObject()->position(p2);
      manualObject()->colour(color_);

      p1 = p2;
    }

    manualObject()->end();
  }
  else
  {
    manualObject()->begin(material()->getName(), Ogre::RenderOperation::OT_LINE_STRIP);

    for (double theta = 0; theta <= 2.0 * Ogre::Math::PI; theta += (2.0 * Ogre::Math::PI) / (double)step_)
    {
      double x = point_.x + (radius_ * cos(theta));
      double y = point_.y + (radius_ * sin(theta));
      manualObject()->position(Ogre::Vector3(x, y, 0));
      manualObject()->colour(color_);
    }

    manualObject()->position(Ogre::Vector3(point_.x + radius_, point_.y, 0));
    manualObject()->colour(color_);

    manualObject()->end();
  }
}
