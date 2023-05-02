#ifndef CIRCLE_H
#define CIRCLE_H

#include "shape_manual_object.h"

class Circle : public ShapeManualObject
{
public:
  Circle(Ogre::SceneManager *manager, Ogre::SceneNode *parent_node = NULL);

public:
  virtual void update();

public:
  inline void setPoint(Ogre::Vector3 point) { point_ = point; }
  inline void setColor(const Ogre::ColourValue &c) { color_ = c; }
  inline void setRadius(float radius) { radius_ = radius; }
  inline void setStep(int step) { step_ = step; }
  inline void setFill(bool fill) { fill_ = fill; }

private:
  Ogre::ColourValue color_;
  Ogre::Vector3 point_;
  float radius_;
  int step_;
  bool fill_;
}; // class Circle

typedef std::shared_ptr<Circle> CirclePtr;

#endif // CIRCLE_H
