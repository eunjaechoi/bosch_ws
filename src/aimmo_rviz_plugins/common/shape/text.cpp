#include "text.h"

Text::Text(Ogre::SceneManager *manager, Ogre::SceneNode *parent_node)
    : Shape(manager, parent_node)
{
  height_ = 1;
  color_ = Ogre::ColourValue(1, 1, 1, 1);

  static int count = 0;
  std::stringstream ss;
  ss << "ShapeText" << count;
  movable_text_ = new Ogre::MovableText(ss.str(), "Text");
  movable_text_->setTextAlignment(Ogre::MovableText::H_CENTER, Ogre::MovableText::V_CENTER);
  movable_text_->showOnTop(true);
  sceneNode()->attachObject(movable_text_);
}

Text::~Text()
{
  delete movable_text_;
}

void Text::update()
{
  movable_text_->setCaption(text_);
  movable_text_->setColor(color_);
  movable_text_->setCharacterHeight(height_);
  sceneNode()->setPosition(position_);
}

void Text::setTextAlignment(const Ogre::MovableText::HorizontalAlignment &horizontalAlignment, const Ogre::MovableText::VerticalAlignment &verticalAlignment)
{
  movable_text_->setTextAlignment(horizontalAlignment, verticalAlignment);
}
