//
// Created by wj on 22. 5. 31.
//

// setting about "Displays" panel in rviz

#ifndef DEPALLETIZING_MAPPING_RVIZ_PLUGIN_DEPALLETIZINGMAPDISPLAY_HPP
#define DEPALLETIZING_MAPPING_RVIZ_PLUGIN_DEPALLETIZINGMAPDISPLAY_HPP
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

namespace depalletizing_mapping_rviz_plugin
{
	class DepalletizingMapVisual;
	class DepalletizingMapDisplay : public rviz::MessageFilterDisplay<depalletizing_mapping_msgs::DepalletizingMap>
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
		void processMessage(const depalletizing_mapping_msgs::DepalletizingMap::ConstPtr& msg);

		boost::mutex mMutex;
		// Storage for the list of visuals
		// circular buffer where data gets popped from the front (oldest) and pushed to the back (newest)
		boost::circular_buffer<boost::shared_ptr<DepalletizingMapVisual>> mVisuals;

		// User-editable property variables.
//		rviz::IntProperty* mQueueSizeProperty;
		rviz::RosTopicProperty* mDepalletizingMapTopicProperty;
		rviz::IntProperty* mHistoryLengthProperty;
		rviz::FloatProperty* mAlphaProperty;
		rviz::ColorProperty* mColorProperty;

		u_int32_t mQueueSize;
	};
}

#endif //DEPALLETIZING_MAPPING_RVIZ_PLUGIN_DEPALLETIZINGMAPDISPLAY_HPP
