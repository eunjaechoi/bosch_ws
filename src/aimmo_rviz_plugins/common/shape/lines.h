#ifndef LINES_H
#define LINES_H

#include <vector>
#include "shape_manual_object.h"

using namespace std;

class Lines : public ShapeManualObject
{
public:
  Lines(Ogre::SceneManager *manager, Ogre::SceneNode *parent_node = NULL);

public:
  virtual void update();

public:  
  inline void setPoints(vector<Ogre::Vector3> &points) { points_ = points; }
  inline void addPoint(Ogre::Vector3 p) { points_.push_back(p); }
  inline void setColor(const Ogre::ColourValue &c) { color_ = c; }

private:
  Ogre::ColourValue color_;
  vector<Ogre::Vector3> points_;
}; // class Lines

typedef std::shared_ptr<Lines> LinesPtr;

#endif // LINES_H
