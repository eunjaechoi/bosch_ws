#ifndef POLAR_GRID_DISPLAY_H
#define POLAR_GRID_DISPLAY_H

#include <vector>

#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/display.h>

#include "../common/shape/circle.h"
#include "../common/shape/lines.h"
#include "../common/shape/text.h"

using namespace std;
using namespace rviz;

namespace aimmo_rviz_plugins
{

  class PolarGridDisplay : public Display
  {
    Q_OBJECT
  public:
    PolarGridDisplay();
    virtual ~PolarGridDisplay();

    // Overrides from Display
    virtual void onInitialize();
    virtual void update(float dt, float ros_dt);
    virtual void reset();
    virtual void onEnable();
    virtual void onDisable();

  private Q_SLOTS:
    void updateGridType();
    void updateCellCount();
    void updateColor();

  private:
    int radiusToStep(float r);
    void drawGrid();

    float getViewControllerDistance();

  private:
    EnumProperty *grid_type_property_;
    TfFrameProperty *frame_property_;
    IntProperty *cell_count_property_;
    FloatProperty *zoom_ratio_property_;
    ColorProperty *color_property_;
    FloatProperty *alpha_property_;

    Ogre::SceneNode *scene_node_;
    vector<CirclePtr> draw_circles_;
    vector<LinesPtr> draw_lines_;
    vector<TextPtr> draw_texts_;

    int grid_type_;
    int cell_count_;
    float view_dist_;
    int cell_level_;
    Ogre::ColourValue color_;

  }; // class PolarGridDisplay

} // namespace aimmo_rviz_plugins

#endif
