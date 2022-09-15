//
// Created by wj on 22. 6. 7.
//
#ifndef DEPALLETIZING_MAPPING_RVIZ_PLUGIN_DEPALLETIZINGMAPVISUAL_HPP
#define DEPALLETIZING_MAPPING_RVIZ_PLUGIN_DEPALLETIZINGMAPVISUAL_HPP

#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreSharedPtr.h>

#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/billboard_line.h>

#include <depalletizing_mapping_core/depalletizing_mapping_core.hpp>
#include <depalletizing_mapping_ros/depalletizing_mapping_ros.hpp>
#include <depalletizing_mapping_msgs/DepalletizingMap.h>

namespace Ogre
{
	class Vector3;
	class Quaternion;
	class ColourValue;
}  // namespace Ogre

namespace rviz
{
	class Shape;
}

namespace depalletizing_mapping_rviz_plugin
{
	class DepalletizingMapVisual
	{
	public:
		DepalletizingMapVisual(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode);
		virtual ~DepalletizingMapVisual();

		void SetMessage(const depalletizing_mapping_msgs::DepalletizingMapConstPtr& msg);

		void SetFramePosition(const Ogre::Vector3& position);
		void SetFrameOrientation(const Ogre::Quaternion& orientation);

		void SetColor(float r, float g, float b, float a);
		void SetColor(float a);

	private:
		Ogre::SceneNode* mFrameNode;
		Ogre::SceneManager* mSceneManager;

		boost::shared_ptr<rviz::Shape> mCylinderShape;
		std::vector<boost::shared_ptr<rviz::Shape>> mCylinderShapes;
	};
}



#endif //DEPALLETIZING_MAPPING_RVIZ_PLUGIN_DEPALLETIZINGMAPVISUAL_HPP
