//
// Created by wj on 22. 5. 31.
//

// setting about "Displays" panel in rviz

#ifndef CAMEL_PERCEPTION_HEIGHTMAP_HEIGHTMAPDISPLAY_HPP
#define CAMEL_PERCEPTION_HEIGHTMAP_HEIGHTMAPDISPLAY_HPP
#ifndef Q_MOC_RUN
#include <depalletizing_mapping_ros/depalletizing_mapping_ros.hpp>
#include <depalletizing_mapping_msgs/DepalletizingMap.h>
#include <boost/circular_buffer.hpp>
#include <rviz/message_filter_display.h>

//#include "grid_map_rviz_plugin/modified/message_filter_display.h"
#endif

namespace Ogre
{
	class SceneNode;
}

namespace rviz
{
	class FloatProperty;
	class IntProperty;
	class ColorProperty;
}

namespace heightmap_rviz_plugin
{
	class HeightmapVisual;
	class DepalletizingMapDisplay : public rviz::MessageFilterDisplay<heightmap_msgs::Heightmap>
	{
	Q_OBJECT
	public:
		DepalletizingMapDisplay();
		virtual ~DepalletizingMapDisplay();

	protected:
		virtual void onInitialize();	// This is where we instantiate all the workings of the class.
		virtual void reset();			// Clear the visuals by deleting their objects.

	//  Qt slots get connected to signals indicating changes in the user-editable properties.
	private Q_SLOTS:
		void updateHistoryLength();
		void updateVisualization();

	private:
		// Callback for incoming ROS messages
		void processMessage(const heightmap_msgs::Heightmap::ConstPtr& msg);

		boost::mutex mMutex;
		// Storage for the list of visuals
		// circular buffer where data gets popped from the front (oldest) and pushed to the back (newest)
		boost::circular_buffer<boost::shared_ptr<HeightmapVisual>> mVisuals;

		// User-editable property variables.
//		rviz::IntProperty* mQueueSizeProperty;
		rviz::RosTopicProperty* mHeightmapTopicProperty;
		rviz::IntProperty* mHistoryLengthProperty;
		rviz::FloatProperty* mAlphaProperty;
		rviz::ColorProperty* mColorProperty;

		u_int32_t mQueueSize;
	};
}

#endif //CAMEL_PERCEPTION_HEIGHTMAP_HEIGHTMAPDISPLAY_HPP
