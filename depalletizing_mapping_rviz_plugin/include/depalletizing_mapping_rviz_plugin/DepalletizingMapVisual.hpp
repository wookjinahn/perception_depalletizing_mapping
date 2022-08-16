//
// Created by wj on 22. 6. 7.
//

#ifndef CAMEL_PERCEPTION_HEIGHTMAP_HEIGHTMAPVISUAL_HPP
#define CAMEL_PERCEPTION_HEIGHTMAP_HEIGHTMAPVISUAL_HPP

#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreSharedPtr.h>

#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/billboard_line.h>

#include <depalletizing_mapping_core/depalletizing_mapping_core.hpp>
#include <depalletizing_mapping_ros/depalletizing_mapping_ros.hpp>
#include <heightmap_msgs/Heightmap.h>

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

namespace heightmap_rviz_plugin
{
	class DepalletizingMapVisual
	{
	public:
		DepalletizingMapVisual(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode);
		virtual ~DepalletizingMapVisual();

		void SetMessage(const heightmap_msgs::HeightmapConstPtr& msg);

		void SetFramePosition(const Ogre::Vector3& position);
		void SetFrameOrientation(const Ogre::Quaternion& orientation);

		void SetColor(float r, float g, float b, float a);
		void SetColor(float a);

	private:
		Ogre::SceneNode* mFrameNode;
		Ogre::SceneManager* mSceneManager;

		boost::shared_ptr<rviz::Shape> mCylinderShape;
		std::vector<boost::shared_ptr<rviz::Shape>> mCylinderShapes;
//		boost::shared_ptr<std::vector<rviz::Shape>> mCylinderShapes;
	};
}



#endif //CAMEL_PERCEPTION_HEIGHTMAP_HEIGHTMAPVISUAL_HPP
