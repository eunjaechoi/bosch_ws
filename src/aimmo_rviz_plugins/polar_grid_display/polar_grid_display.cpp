#include <stdint.h>
#include <cmath>

#include <boost/bind.hpp>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/grid.h>
#include <rviz/properties/parse_color.h>
#include <rviz/properties/property.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/view_manager.h>

#include "polar_grid_display.h"

namespace aimmo_rviz_plugins
{
  const std::vector<float> CELL_LEVELS = {0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1, 2.5, 5, 10, 25, 50, 100};

  PolarGridDisplay::PolarGridDisplay()
      : Display()
  {
    cell_level_ = 6;
    view_dist_ = 10;
    grid_type_property_ = new EnumProperty("Type", "Grid", "Draw Grid Type", this, SLOT(updateGridType()));
    grid_type_property_->addOption("Grid", 0);
    grid_type_property_->addOption("Radius", 1);

    frame_property_ = new TfFrameProperty("Reference Frame", TfFrameProperty::FIXED_FRAME_STRING,
                                          "The TF frame this grid will use for its origin.",
                                          this, 0, true);

    cell_count_property_ = new IntProperty("Cell Count", 100,
                                           "The number of cells to draw in the plane of the grid.",
                                           this, SLOT(updateCellCount()));
    cell_count_property_->setMin(1);

    color_property_ = new ColorProperty("Color", Qt::gray,
                                        "The color of the grid lines.",
                                        this, SLOT(updateColor()));
    alpha_property_ = new FloatProperty("Alpha", 0.5f,
                                        "The amount of transparency to apply to the grid lines.",
                                        this, SLOT(updateColor()));
    alpha_property_->setMin(0.0f);
    alpha_property_->setMax(1.0f);
  }

  PolarGridDisplay::~PolarGridDisplay()
  {
    scene_manager_->clearScene();
  }

  void PolarGridDisplay::onInitialize()
  {
    Ogre::SceneNode *rootNode = scene_manager_->getRootSceneNode();
    scene_node_ = rootNode->createChildSceneNode();

    frame_property_->setFrameManager(context_->getFrameManager());
  }

  void PolarGridDisplay::update(float dt, float ros_dt)
  {
    QString qframe = frame_property_->getFrame();
    std::string frame = qframe.toStdString();

    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if (context_->getFrameManager()->getTransform(frame, ros::Time(), position, orientation))
    {
      scene_node_->setPosition(position);
      scene_node_->setOrientation(orientation);
      setStatus(StatusProperty::Ok, "Transform", "Transform OK");
    }
    else
    {
      std::string error;
      if (context_->getFrameManager()->transformHasProblems(frame, ros::Time(), error))
      {
        setStatus(StatusProperty::Error, "Transform", QString::fromStdString(error));
      }
      else
      {
        setStatus(StatusProperty::Error, "Transform",
                  "Could not transform from [" + qframe + "] to [" + fixed_frame_ + "]");
      }
    }

    // compute cell size
    auto new_view_dist = getViewControllerDistance();

    if (std::abs(new_view_dist - view_dist_) > 0.1)
    {
      const float zoom_ratio = 10;
      auto new_cell_size = std::max(CELL_LEVELS.front(), new_view_dist / zoom_ratio);

      int new_cell_level = cell_level_;

      if (new_view_dist < view_dist_)
      {
        for (int i = cell_level_; i > 0; i--)
        {
          if (CELL_LEVELS[i - 1] < new_cell_size)
          {
            new_cell_level = i;
            break;
          }
        }
      }
      else
      {
        for (int i = cell_level_; i < CELL_LEVELS.size() - 1; i++)
        {
          if (CELL_LEVELS[i + 1] > new_cell_size)
          {
            new_cell_level = i;
            break;
          }
        }
      }

      // need cell size change ?
      if (new_cell_level != cell_level_)
      {
        cell_level_ = new_cell_level;
        drawGrid();
      }

      view_dist_ = new_view_dist;
    }
  }

  void PolarGridDisplay::reset()
  {
    draw_lines_.clear();
    draw_circles_.clear();
    draw_texts_.clear();

    scene_node_->removeAllChildren();
  }

  void PolarGridDisplay::onEnable()
  {
    QColor color = color_property_->getColor();
    color.setAlphaF(alpha_property_->getFloat());
    color_ = qtToOgre(color);
    cell_count_ = cell_count_property_->getInt();

    drawGrid();

    scene_node_->setVisible(true);
  }

  void PolarGridDisplay::onDisable()
  {
    scene_node_->setVisible(false);
  }

  void PolarGridDisplay::updateGridType()
  {
    grid_type_ = grid_type_property_->getOptionInt();
    drawGrid();
  }

  void PolarGridDisplay::updateColor()
  {
    QColor color = color_property_->getColor();
    color.setAlphaF(alpha_property_->getFloat());
    color_ = qtToOgre(color);
    drawGrid();
  }

  void PolarGridDisplay::updateCellCount()
  {
    cell_count_ = cell_count_property_->getInt();
    drawGrid();
  }

  int PolarGridDisplay::radiusToStep(float r)
  {
    float ds = 0.1;
    float dth = ds / r;
    return floor(2. * M_PI / dth);
  }

  void PolarGridDisplay::drawGrid()
  {
    float cell_size = CELL_LEVELS[cell_level_];

    draw_circles_.clear();
    draw_lines_.clear();
    draw_texts_.clear();

    const float text_height = cell_size * 0.2;

    int num_precision = 0;
    if (cell_size < 0.1)
    {
      num_precision = 2;
    }
    else if (cell_size < 1)
    {
      num_precision = 1;
    }

    if (grid_type_ == 0)

    { // Grid
      float x_min = -cell_count_ * cell_size * 0.5;
      float y_min = -cell_count_ * cell_size * 0.5;
      float x_max = x_min + cell_count_ * cell_size;
      float y_max = y_min + cell_count_ * cell_size;
      float x1, y1, x2, y2;
      for (int i = 0; i < cell_count_ + 1; i++)
      {
        // x-direction

        x1 = x_min + (i * cell_size);
        y1 = y_min;
        x2 = x1;
        y2 = y_max;

        LinesPtr l1(new Lines(scene_manager_, scene_node_));
        l1->addPoint(Ogre::Vector3(x1, y1, 0));
        l1->addPoint(Ogre::Vector3(x2, y2, 0));
        l1->setColor(color_);
        l1->update();
        draw_lines_.push_back(l1);

        // y-direction
        x1 = x_min;
        y1 = y_min + (i * cell_size);
        x2 = x_max;
        y2 = y1;

        LinesPtr l2(new Lines(scene_manager_, scene_node_));
        l2->addPoint(Ogre::Vector3(x1, y1, 0));
        l2->addPoint(Ogre::Vector3(x2, y2, 0));
        l2->setColor(color_);
        l2->update();
        draw_lines_.push_back(l2);
      }
    }
    else // Radius
    {
      for (int i = 0; i < cell_count_; i++)
      {
        float r = (i + 1) * cell_size;

        CirclePtr circle(new Circle(scene_manager_, scene_node_));
        circle->setRadius(r);
        circle->setStep(radiusToStep(r));
        circle->setColor(color_);
        circle->update();
        draw_circles_.push_back(circle);

        // std::stringstream ss;
        // ss << std::fixed << std::setprecision(num_precision) << r;
        // std::string text = ss.str() + "m";

        // TextPtr text_top(new Text(scene_manager_, scene_node_));
        // text_top->setColor(color_);
        // text_top->setText(text);
        // text_top->setHeight(text_height);
        // text_top->setPosition(r, 0, 0);
        // text_top->update();
        // draw_texts_.push_back(text_top);

        // TextPtr text_bottom(new Text(scene_manager_, scene_node_));
        // text_bottom->setColor(color_);
        // text_bottom->setText(text);
        // text_bottom->setHeight(text_height);
        // text_bottom->setPosition(-r, 0, 0);
        // text_bottom->update();
        // draw_texts_.push_back(text_bottom);

        // TextPtr text_left(new Text(scene_manager_, scene_node_));
        // text_left->setColor(color_);
        // text_left->setText(text);
        // text_left->setHeight(text_height);
        // text_left->setPosition(0, r, 0);
        // text_left->update();
        // draw_texts_.push_back(text_left);

        // TextPtr text_right(new Text(scene_manager_, scene_node_));
        // text_right->setColor(color_);
        // text_right->setText(text);
        // text_right->setHeight(text_height);
        // text_right->setPosition(0, -r, 0);
        // text_right->update();
        // draw_texts_.push_back(text_right);
      }
    }

    context_->queueRender();
  }

  float PolarGridDisplay::getViewControllerDistance()
  {
    auto view_ctrl = context_->getViewManager()->getCurrent();
    for (int i = 0; i < view_ctrl->numChildren(); i++)
    {
      auto prop = view_ctrl->childAt(i);
      if (prop->getName() == "Distance")
      {
        return prop->getValue().toFloat();
        break;
      }
    }
    return -1;
  }

} // namespace aimmo_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(aimmo_rviz_plugins::PolarGridDisplay, rviz::Display)
