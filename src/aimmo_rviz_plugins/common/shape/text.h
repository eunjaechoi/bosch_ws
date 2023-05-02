#ifndef TEXT_H
#define TEXT_H

#include <string>
#include "shape.h"
#include "ogre_helper/movable_text.h"

using namespace std;

class Text : public Shape
{
public:
  Text(Ogre::SceneManager *manager, Ogre::SceneNode *parent_node = NULL);
  virtual ~Text();

public:
  virtual void update();

public:
  inline void setPosition(float x, float y, float z) { position_ = Ogre::Vector3(x, y, z); }
  inline void setColor(const Ogre::ColourValue &c) { color_ = c; }
  inline void setText(string text) { text_ = text; }
  inline void setHeight(float height) { height_ = height; }
  inline void setTextAlignment(const Ogre::MovableText::HorizontalAlignment &horizontalAlignment, const Ogre::MovableText::VerticalAlignment &verticalAlignment);

private:
  Ogre::MovableText *movable_text_;
  Ogre::ColourValue color_;
  Ogre::Vector3 position_;
  string text_;
  float height_;
};

typedef std::shared_ptr<Text> TextPtr;

#endif // TEXT_H
